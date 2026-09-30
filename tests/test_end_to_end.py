"""The whole program against the synthetic device, with planted answers.

Everything here runs atkdaq as a separate process, exactly as ATK will, with
the synthetic device in fast mode (as fast as the test reads). The device
plants per-channel stream offsets, receiver gains/phases/fractional delays,
an on-air emitter with known per-element phases, sample loss on one channel,
retunes that re-randomise every tuner phase — and the frames that come out
must carry every planted answer back.

This is how the DAQ is proven before the KrakenSDR arrives; what it cannot
prove (USB timing on Windows, the fork's GPIO numbering on the real board,
the R820T's settle times) is exactly the list T1/T2 on the hardware answers.
"""

from __future__ import annotations

import subprocess
import threading
import time

import numpy as np
import pytest

from atkdaq import frame as F
from atkdaq.client import open_stream, parse_status

PLANT = ["--set", "synth.start=0,17,-5,120,3",
         "--set", "synth.frac=0,0.21,-0.33,0.12,0.44",
         "--set", "synth.gain_db=0,-1,0.5,-0.4,1.2",
         "--set", "synth.phase_deg=0,40,-100,170,20"]
FAST = ["--device", "synth", "--set", "synth.fast=1"]
WANT_DELAYS = [0, -17, 5, -120, -3]            # start[0] - start[i]
WANT_FRAC = [0, 0.21, -0.33, 0.12, 0.44]
WANT_WDB = [0, 1.0, -0.5, 0.4, -1.2]
WANT_WDEG = [0, -40, 100, -170, -20]


def run(exe, args, timeout=120):
    """Run to completion; (returncode, headers, events)."""
    p = subprocess.run([str(exe)] + args, capture_output=True, timeout=timeout,
                       stdin=subprocess.DEVNULL)
    import io
    headers = [fr.header for fr in F.read_frames(io.BytesIO(p.stdout))]
    events = [e for e in (parse_status(l) for l in p.stderr.decode().splitlines()) if e]
    return p.returncode, headers, events, p.stdout


def ev(events, name):
    return [e for e in events if e.get("event") == name]


# ---------------------------------------------------------------------------

def test_planted_delays_weights_and_fractional_delays_come_back(exe):
    rc, hs, evs, _ = run(exe, FAST + PLANT + ["--frames", "60", "--no-stdin"])
    assert rc == 0, evs
    s = ev(evs, "sync")
    assert s, "no sync event"
    assert [int(v) for v in s[0]["delays"].split(",")] == WANT_DELAYS
    good = [h for h in hs if h.usable_for_df]
    assert len(good) > 30
    h = good[-1]
    assert list(h.delays) == WANT_DELAYS
    assert np.allclose(h.frac_delays, WANT_FRAC, atol=0.005)
    wdb = 20 * np.log10(np.abs(h.weights))
    wdeg = np.degrees(np.angle(h.weights))
    assert np.allclose(wdb, WANT_WDB, atol=0.05)
    dd = (wdeg - np.array(WANT_WDEG) + 180) % 360 - 180
    assert np.all(np.abs(dd) < 0.3), wdeg
    assert h.spread_deg is not None and h.spread_deg < 1.0
    assert h.sync_ptn_db > 20


def test_header_invariants_across_a_run(exe):
    rc, hs, evs, _ = run(exe, FAST + PLANT + ["--frames", "60", "--no-stdin"])
    assert rc == 0
    assert all(h.has(F.F_SYNTHETIC) for h in hs), "a synthetic frame without the SYNTHETIC flag"
    assert hs[0].has(F.F_SEGMENT_START)
    for a, b in zip(hs, hs[1:]):
        assert b.seq == a.seq + 1
        assert b.stream_sample0 == a.stream_sample0 + a.samples_per_ch
        assert b.t_utc_ns > a.t_utc_ns
    # before the first calibration nothing is usable, and says why
    first_good = next(i for i, h in enumerate(hs) if h.usable_for_df)
    assert all(h.has(F.F_MISALIGNED) or h.has(F.F_NOISE_ON) for h in hs[:first_good])
    # every noise-source frame is marked, and none of them is usable
    assert any(h.has(F.F_NOISE_ON) for h in hs)
    assert not any(h.usable_for_df for h in hs if h.has(F.F_NOISE_ON))
    # passport identifies the calibration
    ps = {h.passport for h in hs if h.usable_for_df}
    assert ps == {1}


def test_after_calibration_only_the_on_air_steering_phases_remain(exe):
    """The emitter arrives at the five elements with planted phases (the
    steering vector); the receivers add their own gains, phases and
    fractional delays on top. After the frame's calibration is applied —
    weight AND slope — what is left must be the steering vector alone."""
    steer = [0, 72, 144, 216, 288]
    f0 = 300_000.0
    args = FAST + PLANT + ["--set", f"synth.emitter={int(f0)},-20," + ",".join(map(str, steer)),
                           "--frames", "40", "--no-stdin"]
    p = subprocess.run([str(exe)] + args, capture_output=True, timeout=120, stdin=subprocess.DEVNULL)
    import io
    frames = [fr for fr in F.read_frames(io.BytesIO(p.stdout)) if fr.usable_for_df]
    assert frames
    fr = frames[-1]
    y = fr.calibrated()
    n = y.shape[1]
    Y = np.fft.fft(y, axis=1)
    f = np.fft.fftfreq(n, 1 / fr.header.fs_hz)
    band = np.abs(f - f0) < 5000
    rel = [np.degrees(np.angle(np.sum(Y[i, band] * np.conj(Y[0, band])))) for i in range(5)]
    want = np.array(steer) - steer[0]
    err = (np.array(rel) - want + 180) % 360 - 180
    assert np.all(np.abs(err) < 1.0), (rel, want)
    # and WITHOUT the fractional-delay slope, the error is visibly larger:
    # the weight alone is right only at band centre (the trap the header warns about)
    x = fr.iq * fr.header.weights[:, None]
    X = np.fft.fft(x, axis=1)
    rel2 = [np.degrees(np.angle(np.sum(X[i, band] * np.conj(X[0, band])))) for i in range(5)]
    err2 = (np.array(rel2) - want + 180) % 360 - 180
    assert np.max(np.abs(err2)) > 10.0


def test_a_channel_that_loses_samples_is_caught_and_resynced(exe):
    args = FAST + PLANT + ["--set", "synth.drop=2,3.0,5000", "--mode", "mobile",
                           "--seconds", "8", "--no-stdin"]
    rc, hs, evs, _ = run(exe, args)
    assert rc == 0
    mis = ev(evs, "misaligned")
    assert mis, "the loss was not detected"
    assert "2:" in mis[0]["channels_samples"]
    lost = int(mis[0]["channels_samples"].split("2:")[1].split(",")[0])
    assert abs(lost - 5000) < 1000
    syncs = ev(evs, "sync")
    assert len(syncs) >= 2
    after = [int(v) for v in syncs[-1]["delays"].split(",")]
    assert after[2] == WANT_DELAYS[2] - 5000, after
    assert [a for i, a in enumerate(after) if i != 2] == [d for i, d in enumerate(WANT_DELAYS) if i != 2]
    # WHERE: the loss is located to one USB transfer (131072 samples); the
    # device dropped it at stream time 3.0 s on its own clock
    since = int(mis[0]["since"])
    assert abs(since - 3.0 * 2.4e6) < 2 * 131072, since
    # frames sent after the detection are never usable until the resync
    from_s0 = int(mis[0]["from_s0"])
    new_passport = max(h.passport for h in hs)
    between = [h for h in hs if h.stream_sample0 >= from_s0 and h.passport < new_passport]
    assert between and not any(h.usable_for_df for h in between)
    final = hs[-1]
    assert final.usable_for_df and list(final.delays) == after


def test_static_mode_waits_for_quiet_or_five_seconds(exe):
    args = FAST + PLANT + ["--set", "synth.drop=1,2.0,20000", "--seconds", "12", "--no-stdin"]
    rc, hs, evs, _ = run(exe, args)
    mis = ev(evs, "misaligned")
    assert mis
    t_mis = next(i for i, e in enumerate(evs) if e.get("event") == "misaligned")
    later_syncs = [e for e in evs[t_mis:] if e.get("event") == "sync"]
    assert later_syncs, "static mode never resynced (it should after quiet_wait_s)"
    assert int(later_syncs[0]["delays"].split(",")[1]) == WANT_DELAYS[1] - 20000


def test_retunes_teach_the_policy_the_fast_retune_hypothesis(exe):
    with open_stream(FAST + PLANT + ["--seconds", "60"], exe=exe) as s:
        hdrs = []
        stop = threading.Event()

        def reader():
            for fr in s.frames():
                hdrs.append(fr.header)
                if stop.is_set():
                    break

        t = threading.Thread(target=reader, daemon=True)
        t.start()
        assert s.wait_event("sync", 30)
        kinds = []
        for k in range(5):
            mark = len(s.events)
            s.control.tune(446e6 + (k + 1) * 1e6)
            got = None
            end = time.monotonic() + 30
            while time.monotonic() < end and got is None:
                for e in s.events[mark:]:
                    if e.get("event") in ("sync", "cal", "proc_fail"):
                        got = e
                        break
                time.sleep(0.05)
            assert got is not None and got["event"] != "proc_fail", got
            kinds.append(got["event"])
            if got["event"] == "sync":
                assert got["changed"] == "0", "a retune moved the delays with the hypothesis true"
        stop.set()
    # three full syncs prove the hypothesis, then retunes cost only a cal
    assert kinds[:3] == ["sync", "sync", "sync"]
    assert kinds[3:] == ["cal", "cal"]
    segs = [h for h in hdrs if h.has(F.F_SEGMENT_START)]
    assert len(segs) >= 5
    assert any(h.has(F.F_RETUNE) for h in hdrs)
    fcs = sorted({h.fc_hz for h in hdrs})
    assert int(446e6 + 5e6) in fcs


def test_when_retunes_move_the_delays_every_retune_gets_a_full_sync(exe):
    with open_stream(FAST + PLANT + ["--set", "synth.retune_moves_delays=1", "--seconds", "60"],
                     exe=exe) as s:
        t = threading.Thread(target=lambda: [None for _ in s.frames()], daemon=True)
        t.start()
        assert s.wait_event("sync", 30)
        changed = []
        for k in range(4):
            mark = len(s.events)
            s.control.tune(446e6 + (k + 1) * 1e6)
            e = None
            end = time.monotonic() + 30
            while time.monotonic() < end and e is None:
                e = next((x for x in s.events[mark:] if x.get("event") in ("sync", "cal")), None)
                time.sleep(0.05)
            assert e is not None
            changed.append((e["event"], e.get("changed")))
    # with the offsets moving 0..2 samples on a retune, some retune moves one;
    # after that the policy never trusts a cal-only retune again
    assert all(k == "sync" for k, _ in changed), changed


def test_a_cal_that_finds_the_delays_moved_syncs_instead_of_applying(exe):
    """Forced cal-only retunes (retune_policy=2) over a device whose retunes
    DO move the delays: the cal sees a whole-sample residual, refuses to
    apply it, and a sync with changed delays follows at once. Without this
    the weights would be right at one frequency and the frames a sample or
    two out of alignment, with nothing in the header to say so."""
    with open_stream(FAST + PLANT + ["--set", "synth.retune_moves_delays=1",
                                     "--set", "retune_policy=2", "--seconds", "60"], exe=exe) as s:
        hdrs = []
        t = threading.Thread(target=lambda: [hdrs.append(fr.header) for fr in s.frames()],
                             daemon=True)
        t.start()
        assert s.wait_event("sync", 30)
        moved = None
        for k in range(6):
            mark = len(s.events)
            s.control.tune(446e6 + (k + 1) * 1e6)
            end = time.monotonic() + 30
            outcome = None
            while time.monotonic() < end and outcome is None:
                outcome = next((x for x in s.events[mark:]
                                if x.get("event") in ("cal", "delays_moved", "proc_fail")), None)
                time.sleep(0.05)
            assert outcome is not None and outcome["event"] != "proc_fail", outcome
            if outcome["event"] == "delays_moved":
                moved = mark
                break
        assert moved is not None, "no retune moved a delay in six (p < 1e-11 by chance)"
        end = time.monotonic() + 30
        sync = None
        while time.monotonic() < end and sync is None:
            sync = next((x for x in s.events[moved:] if x.get("event") == "sync"), None)
            time.sleep(0.05)
        assert sync is not None, "the moved delays were not resynced"
        assert sync["changed"] == "1"
        time.sleep(0.5)
    # the moved-delay cal was never applied: no usable frame is aligned worse
    # than the header promises ("within one sample period"), i.e. no frame
    # carries a fractional delay beyond ATKDAQ_FRAC_WHOLE
    fr = np.array([h.frac_delays for h in hdrs if h.usable_for_df])
    assert fr.size and np.max(np.abs(fr)) <= 0.75, np.max(np.abs(fr))


def test_a_stalled_consumer_costs_frames_never_alignment(exe):
    s = open_stream(FAST + PLANT + ["--seconds", "20"], exe=exe)
    it = s.frames()
    got = []
    try:
        for fr in it:
            got.append(fr.header)
            if len(got) == 40:
                time.sleep(3.0)          # stop reading: the queue fills, frames drop
            if len(got) >= 120:
                break
    finally:
        s.close()
    gaps = [(a, b) for a, b in zip(got, got[1:]) if b.seq != a.seq + 1]
    assert gaps, "the stall dropped no frames (the test did not stall the program)"
    for a, b in gaps:
        missing = b.seq - a.seq - 1
        assert b.stream_sample0 == a.stream_sample0 + (missing + 1) * a.samples_per_ch
        assert b.has(F.F_DISCONTINUITY)
    good = [h for h in got if h.usable_for_df]
    assert good and all(list(h.delays) == WANT_DELAYS for h in good)
    assert s.last("stall") is not None


def test_noise_hold_and_release(exe):
    with open_stream(FAST + PLANT + ["--seconds", "30"], exe=exe) as s:
        hdrs = []
        t = threading.Thread(target=lambda: [hdrs.append(fr.header) for fr in s.frames()],
                             daemon=True)
        t.start()
        assert s.wait_event("sync", 30)
        mark = len(s.events)
        s.control.noise("on")
        assert s.wait_event("noise", 10, after=mark)
        time.sleep(0.5)
        n_before = len(hdrs)
        time.sleep(0.5)
        held = hdrs[n_before:]
        mark = len(s.events)
        s.control.noise("auto")
        off = s.wait_event("noise", 10, after=mark)
    assert held and all(h.has(F.F_NOISE_ON) for h in held[2:])
    assert off and off["state"] == "off"


def test_record_then_play_reproduces_the_stream_byte_for_byte(exe, tmp_path):
    out = tmp_path / "rec.atkq"
    p = subprocess.run([str(exe), "record", str(out)] + FAST + PLANT + ["--frames", "30"],
                       capture_output=True, timeout=120)
    assert p.returncode == 0, p.stderr.decode()
    data = out.read_bytes()
    assert len(data) > 0
    q = subprocess.run([str(exe), "play", str(out), "--fast"], capture_output=True, timeout=60)
    assert q.returncode == 0
    assert q.stdout == data


def test_play_survives_garbage_in_the_file(exe, tmp_path):
    out = tmp_path / "rec.atkq"
    subprocess.run([str(exe), "record", str(out)] + FAST + ["--frames", "6"],
                   capture_output=True, timeout=60, check=True)
    data = bytearray(out.read_bytes())
    fb = len(data) // 6
    data[fb + 10:fb + 40] = b"\x00" * 30          # break frame 2's header
    bad = tmp_path / "bad.atkq"
    bad.write_bytes(bytes(data))
    q = subprocess.run([str(exe), "play", str(bad), "--fast"], capture_output=True, timeout=60)
    import io
    frames = list(F.read_frames(io.BytesIO(q.stdout)))
    assert len(frames) == 5
    assert b"event=resync" in q.stderr


def test_configuration_errors_name_the_setting(exe):
    p = subprocess.run([str(exe), "run", "--set", "gian_tenths_db=280"], capture_output=True, timeout=30)
    assert p.returncode == 2
    assert b"unknown setting 'gian_tenths_db'" in p.stderr
    p = subprocess.run([str(exe), "run", "--set", "fs_hz=5000000"], capture_output=True, timeout=30)
    assert p.returncode == 2 and b"fs_hz" in p.stderr


def test_a_missing_kraken_is_reported_plainly(exe):
    p = subprocess.run([str(exe), "run", "--device", "kraken", "--no-stdin"], capture_output=True,
                       timeout=30)
    assert p.returncode in (3, 4)
    msg = p.stderr.decode()
    assert "no RTL-SDR devices found" in msg or "without librtlsdr" in msg or "not found" in msg


def test_quit_on_stdin_and_on_eof(exe):
    s = open_stream(FAST + ["--seconds", "60"], exe=exe)
    it = s.frames()
    for _ in range(3):
        next(it)
    s.control.quit()
    for _ in it:
        pass
    assert s.proc.wait(20) == 0
    s2 = open_stream(FAST + ["--seconds", "60"], exe=exe)
    s2.proc.stdin.close()                          # EOF on stdin = quit
    for _ in s2.frames():
        pass
    assert s2.proc.wait(20) == 0
