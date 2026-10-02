#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""T1b — what a retune costs, and whether a calibration can be cached.

Run on the day the Kraken arrives (ATK docs/CUED_DF_PLAN.md §13):

    python bench/t1b_cache_validity.py --device kraken --f0 452e6 --gain 300
    python bench/t1b_cache_validity.py --device synth --set synth.fast=1   # dry run

It drives atkdaq through its stdin, reads every frame, and keeps one record
per CALIBRATION (a new passport): time, fc, gain, delays, weights, fractional
delays, the DAQ's own spread. Four stages, each optional:

  hops     tune f0 → f0+Δ → f0, for each Δ, N times; forced cal after each tune
  age      stay on f0, force a cal every --age-every seconds for --age-minutes
  gain     change gain, cal, change back, cal
  (settle is measured on every hop: command → first frame at the new fc, and
   command → first frame usable for DF)

Then it answers, from the records alone:

  * does a retune keep the integer delays?           (atkdaq §6 — the hypothesis)
  * what is the phase difference between two cals at the SAME fc, separated
    by a retune away and back?  If that is small, a per-frequency calibration
    cache can work; if it is random, the PLL relocks with an arbitrary phase
    and the cache cannot hold PLL phase — only the cables' slope and the gains.
  * what would a cached entry from f0 cost at f0+Δ?  (per-channel phase of
    w(f0+Δ) / w(f0), RMS over channels)
  * how fast do the weights drift at a fixed fc?     (age → spread)
  * what does a gain change do to the weights?
  * settle, per hop size, and the cal's blind time as the DAQ reports it.

Everything is written to a JSON (--out) so the analysis can be re-run with
--analyse FILE and so the numbers become the scheduler's constants.
No hardware here is assumed to behave; the point is to find out.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "python"))

from atkdaq.client import open_stream  # noqa: E402
from atkdaq.frame import CAL_CALIBRATED, F_RETUNE  # noqa: E402


# ---------------------------------------------------------------------------
# recording
# ---------------------------------------------------------------------------

def _rec(hdr, t_host: float, stage: str, note: str = "") -> dict:
    return {
        "stage": stage, "note": note, "t_host": t_host, "t_utc": hdr.t_utc,
        "passport": hdr.passport, "fc_hz": hdr.fc_hz, "gain_tenths": hdr.gain_tenths_db,
        "delays": hdr.delays.tolist(),
        "frac": hdr.frac_delays.tolist(),
        "w_re": [float(x) for x in hdr.weights.real],
        "w_im": [float(x) for x in hdr.weights.imag],
        "spread_deg": hdr.spread_deg, "cal_age_ms": hdr.cal_age_ms,
        "flags": hdr.flag_names,
    }


class Driver:
    """Sends commands, watches frames, keeps one record per new passport."""

    def __init__(self, daq, timeout: float):
        self.daq = daq
        self.it = daq.frames()
        self.timeout = timeout
        self.records: list[dict] = []
        self.hops: list[dict] = []
        self.last_passport = -1
        self.last_fc = None

    def _next(self):
        return next(self.it)

    def wait_calibrated(self, stage: str, note: str = "", *, new_passport: bool = True,
                        record: bool = True):
        """Block until a CALIBRATED, usable frame (with a new passport when
        asked). Returns (header, t_host). `record=False` observes without
        adding to the per-calibration record list."""
        t_end = time.monotonic() + self.timeout
        while time.monotonic() < t_end:
            fr = self._next()
            h = fr.header
            if h.cal_state == CAL_CALIBRATED and fr.usable_for_df and (
                    not new_passport or h.passport != self.last_passport):
                self.last_passport = h.passport
                self.last_fc = h.fc_hz
                t_host = time.monotonic()
                if record:
                    self.records.append(_rec(h, t_host, stage, note))
                return h, t_host
        raise TimeoutError(f"no calibrated frame within {self.timeout}s ({stage} {note})")

    def hop(self, fc_hz: float, stage: str, note: str):
        """tune → (first frame at new fc) → (first usable frame); forced cal.
        Records settle times and whether the delays moved."""
        before = self.records[-1] if self.records else None
        t0 = time.monotonic()
        self.daq.control.tune(fc_hz)
        t_fc = None
        t_end = t0 + self.timeout
        # first frame whose fc is the new one and which is not flagged RETUNE
        while time.monotonic() < t_end:
            fr = self._next()
            h = fr.header
            if h.fc_hz == int(round(fc_hz)) and not (h.flags & F_RETUNE):
                t_fc = time.monotonic()
                break
        if t_fc is None:
            raise TimeoutError(f"tuner never reported {fc_hz/1e6:.3f} MHz")
        # The DAQ's own retune policy (sync+cal, or cal only once the delays
        # have proven to survive) runs now; the first usable frame after the
        # tune is what a retune COSTS the scheduler. Measured before anything
        # is forced.
        h_policy, t_policy = self.wait_calibrated(stage, note, new_passport=False,
                                                  record=False)
        # Then a forced cal, so every hop ends on a fresh passport of its own
        # whatever the policy did — that passport is what the cache analysis
        # compares.
        self.daq.control.cal()
        h, t_cal = self.wait_calibrated(stage, note)
        self.hops.append({
            "stage": stage, "note": note, "fc_hz": int(round(fc_hz)),
            "settle_fc_ms": (t_fc - t0) * 1e3,
            "settle_usable_ms": (t_policy - t0) * 1e3,
            "forced_cal_ms": (t_cal - t_policy) * 1e3,
            "policy_passport_new": h_policy.passport != (before or {}).get("passport"),
            "delays_before": before["delays"] if before else None,
            "delays_after": h.delays.tolist(),
            "delays_moved": bool(before and before["delays"] != h.delays.tolist()),
            "spread_deg": h.spread_deg,
        })
        return h


# ---------------------------------------------------------------------------
# analysis (pure; works on a saved JSON)
# ---------------------------------------------------------------------------

def _w(rec: dict) -> np.ndarray:
    return np.array(rec["w_re"]) + 1j * np.array(rec["w_im"])


def phase_rms_deg(a: dict, b: dict) -> float:
    """RMS over channels 1.. of the phase of w_b / w_a, with channel 0's
    phase removed (the weights are relative already; this guards a
    convention change)."""
    r = _w(b) / _w(a)
    r = r / (r[0] / abs(r[0]))
    ph = np.degrees(np.angle(r[1:]))
    return float(math.sqrt(np.mean(ph ** 2)))


def amp_rms_db(a: dict, b: dict) -> float:
    r = np.abs(_w(b) / _w(a))
    return float(math.sqrt(np.mean((20 * np.log10(r[1:])) ** 2)))


def _q(xs: list[float]) -> dict:
    if not xs:
        return {}
    xs = sorted(xs)
    return {"n": len(xs), "min": xs[0], "median": statistics.median(xs),
            "p95": xs[min(len(xs) - 1, int(round(0.95 * (len(xs) - 1))))],
            "max": xs[-1]}


def analyse(data: dict) -> dict:
    recs = data["records"]
    hops = data["hops"]
    f0 = int(round(data["args"]["f0"]))
    out: dict = {"f0_hz": f0}

    # 1. delays survive a retune?
    moved = [h for h in hops if h["delays_before"] is not None]
    out["retunes_measured"] = len(moved)
    out["retunes_that_moved_delays"] = sum(1 for h in moved if h["delays_moved"])
    out["delays_survive_retune"] = (out["retunes_measured"] > 0
                                    and out["retunes_that_moved_delays"] == 0)

    # 2. settle by hop size
    by_size: dict[str, list] = {}
    for h in hops:
        by_size.setdefault(h["note"], []).append(h)
    out["settle_ms"] = {
        k: {"to_new_fc": _q([x["settle_fc_ms"] for x in v]),
            "to_usable": _q([x["settle_usable_ms"] for x in v]),
            "forced_cal": _q([x["forced_cal_ms"] for x in v])}
        for k, v in by_size.items()}

    # 3. SAME fc, separated by a retune away and back: is PLL phase repeatable?
    at_f0 = [r for r in recs if r["stage"] == "hops" and r["fc_hz"] == f0]
    back = []
    for a, b in zip(at_f0, at_f0[1:]):
        back.append(phase_rms_deg(a, b))
    out["same_fc_after_retune_phase_rms_deg"] = _q(back)
    out["pll_phase_repeatable"] = bool(back) and statistics.median(back) < 5.0

    # 4. a cached f0 entry applied at f0+Δ (only meaningful if 3 says yes;
    #    reported anyway, since it also measures the slope model)
    cached: dict[str, list] = {}
    prev_f0 = None
    for r in recs:
        if r["stage"] != "hops":
            continue
        if r["fc_hz"] == f0:
            prev_f0 = r
            continue
        if prev_f0 is not None:
            d = r["fc_hz"] - f0
            key = f"{d/1e3:+.0f} kHz"
            cached.setdefault(key, []).append(
                {"phase_rms_deg": phase_rms_deg(prev_f0, r),
                 "amp_rms_db": amp_rms_db(prev_f0, r)})
    out["cached_entry_at_offset"] = {
        k: {"phase_rms_deg": _q([x["phase_rms_deg"] for x in v]),
            "amp_rms_db": _q([x["amp_rms_db"] for x in v])}
        for k, v in cached.items()}

    # 5. ageing at fixed fc
    age = [r for r in recs if r["stage"] == "age"]
    if age:
        a0 = age[0]
        out["age_drift"] = [
            {"age_s": round(r["t_host"] - a0["t_host"], 1),
             "phase_rms_deg": round(phase_rms_deg(a0, r), 2),
             "daq_spread_deg": r["spread_deg"]} for r in age]
        over = [x for x in out["age_drift"] if x["phase_rms_deg"] > 2.0]
        out["age_until_2deg_s"] = over[0]["age_s"] if over else None

    # 6. gain change
    g = [r for r in recs if r["stage"] == "gain"]
    if len(g) >= 2:
        out["gain_change_phase_rms_deg"] = phase_rms_deg(g[0], g[1])
        out["gain_change_amp_rms_db"] = amp_rms_db(g[0], g[1])

    # 7. the DAQ's own numbers
    out["daq_spread_deg"] = _q([r["spread_deg"] for r in recs
                                if r["spread_deg"] is not None])
    out["cal_blind_ms_reported"] = data.get("cal_blind_ms")
    return out


def verdict(an: dict) -> str:
    lines = [f"T1b verdict (f0 = {an['f0_hz']/1e6:.3f} MHz)", ""]
    if an["retunes_measured"]:
        lines.append(
            f"delays after a retune: {'KEPT' if an['delays_survive_retune'] else 'MOVED'} "
            f"({an['retunes_that_moved_delays']} of {an['retunes_measured']} moved) → "
            + ("retune = settle + cal only (atkdaq §6 holds)" if an["delays_survive_retune"]
               else "retune = settle + sync + cal (hypothesis refuted)"))
    s = an.get("same_fc_after_retune_phase_rms_deg") or {}
    if s:
        rep = an["pll_phase_repeatable"]
        lines.append(
            f"PLL phase at the same fc after a retune away and back: median "
            f"{s['median']:.1f}°, p95 {s['p95']:.1f}° → "
            + ("REPEATABLE: a per-frequency calibration cache CAN hold PLL phase"
               if rep else
               "NOT repeatable: the PLL relocks with an arbitrary phase; a cache "
               "can hold the slope and the gains, not the phase — every retune "
               "needs a cal (noise source) or an in-band reference"))
    for k, v in sorted(an.get("settle_ms", {}).items()):
        if v["to_new_fc"]:
            lines.append(f"settle {k}: to new fc median {v['to_new_fc']['median']:.0f} ms, "
                         f"to usable (the DAQ's own retune policy) median "
                         f"{v['to_usable']['median']:.0f} ms (max {v['to_usable']['max']:.0f}); "
                         f"a forced cal on top: median {v['forced_cal']['median']:.0f} ms")
    for k, v in sorted(an.get("cached_entry_at_offset", {}).items(),
                       key=lambda kv: abs(float(kv[0].split()[0]))):
        if v["phase_rms_deg"]:
            lines.append(f"cached f0 entry applied at {k}: phase RMS median "
                         f"{v['phase_rms_deg']['median']:.1f}°, amp {v['amp_rms_db']['median']:.2f} dB")
    if "age_until_2deg_s" in an:
        a = an["age_until_2deg_s"]
        lines.append("drift at fixed fc: " + (f"exceeds 2° after {a:.0f} s" if a is not None
                                             else f"under 2° for the whole run "
                                                  f"({an['age_drift'][-1]['age_s']:.0f} s)"))
    if "gain_change_phase_rms_deg" in an:
        lines.append(f"a gain change moves the phase by {an['gain_change_phase_rms_deg']:.1f}° RMS, "
                     f"amplitude {an['gain_change_amp_rms_db']:.2f} dB → gain belongs in the key")
    d = an.get("daq_spread_deg") or {}
    if d:
        lines.append(f"the DAQ's post-cal spread: median {d['median']:.2f}°, max {d['max']:.2f}°")
    lines.append("")
    lines.append("scheduler constants (CUED_DF_PLAN.md §13): "
                 "cost_retune_ms = settle to usable (median) for the hop size in use; "
                 "cache_phase = " + ("yes" if an.get("pll_phase_repeatable") else "no") + ".")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--device", default="synth", choices=["kraken", "synth"])
    ap.add_argument("--f0", type=float, default=452.0e6, help="base frequency, Hz")
    ap.add_argument("--offsets", default="100e3,500e3,1e6,300e6",
                    help="comma list of hop sizes in Hz (a large one crosses a tuner band)")
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--gain", type=int, default=None, help="tuner gain, tenths of dB")
    ap.add_argument("--gain-alt", type=int, default=None,
                    help="the other gain for the gain stage (default: --gain + 60)")
    ap.add_argument("--age-minutes", type=float, default=0.0)
    ap.add_argument("--age-every", type=float, default=120.0, help="seconds between cals")
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--set", action="append", default=[], help="atkdaq --set KEY=VALUE")
    ap.add_argument("--out", default=None, help="JSON file (default t1b_<time>.json)")
    ap.add_argument("--analyse", default=None, help="only re-analyse this JSON")
    ap.add_argument("--skip-hops", action="store_true")
    args = ap.parse_args(argv)

    if args.analyse:
        data = json.loads(Path(args.analyse).read_text())
        an = analyse(data)
        print(verdict(an))
        return 0

    daq_args = ["--device", args.device, "--fc", str(int(args.f0)), "--mode", "static"]
    if args.gain is not None:
        daq_args += ["--gain", str(args.gain)]
    for s in args.set:
        daq_args += ["--set", s]
    offsets = [float(x) for x in args.offsets.split(",") if x.strip()]

    data = {"args": vars(args), "daq_args": daq_args, "records": [], "hops": [],
            "started": time.time()}
    with open_stream(daq_args) as daq:
        drv = Driver(daq, args.timeout)
        print("waiting for the first calibration …", flush=True)
        h, _ = drv.wait_calibrated("start")
        print(f"  calibrated at {h.fc_hz/1e6:.3f} MHz, spread {h.spread_deg}°, "
              f"delays {h.delays.tolist()}", flush=True)

        if not args.skip_hops:
            for d in offsets:
                label = f"{d/1e3:+.0f} kHz" if d < 50e6 else f"{d/1e6:+.0f} MHz (band cross)"
                print(f"hops {label} × {args.repeats} …", flush=True)
                for i in range(args.repeats):
                    drv.hop(args.f0 + d, "hops", label)
                    drv.hop(args.f0, "hops", "back to f0")
                print(f"  done; last spread {drv.records[-1]['spread_deg']}°", flush=True)

        if args.age_minutes > 0:
            n = int(args.age_minutes * 60 / args.age_every)
            print(f"ageing: {n} cals every {args.age_every:.0f} s at f0 …", flush=True)
            daq.control.cal()
            drv.wait_calibrated("age", "t0")
            for i in range(n):
                t_next = time.monotonic() + args.age_every
                # keep reading so the pipe never fills
                while time.monotonic() < t_next:
                    try:
                        drv._next()
                    except StopIteration:
                        break
                daq.control.cal()
                drv.wait_calibrated("age", f"+{(i+1)*args.age_every:.0f}s")
                print(f"  {i+1}/{n}", flush=True)

        if args.gain is not None:
            alt = args.gain_alt if args.gain_alt is not None else args.gain + 60
            print(f"gain stage: {args.gain} → {alt} → {args.gain} tenths dB …", flush=True)
            daq.control.cal()
            drv.wait_calibrated("gain", f"gain {args.gain}")
            daq.control.gain(alt)
            daq.control.cal()
            drv.wait_calibrated("gain", f"gain {alt}")
            daq.control.gain(args.gain)
            daq.control.cal()
            drv.wait_calibrated("gain", f"gain {args.gain} again")

        data["records"] = drv.records
        data["hops"] = drv.hops
        ev = daq.last("cal")
        data["cal_blind_ms"] = ev.get("blind_ms") if ev else None

    out = Path(args.out or f"t1b_{time.strftime('%Y%m%d-%H%M%S')}.json")
    out.write_text(json.dumps(data, indent=1))
    an = analyse(data)
    out.with_suffix(".verdict.txt").write_text(verdict(an) + "\n")
    print()
    print(verdict(an))
    print(f"\nraw records → {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
