"""How fast can frames cross the pipe? atkdaq (synthetic device, fast mode)
-> stdout -> python/atkdaq.frame.read_frames, for N seconds of stream.

    python bench/bench_throughput.py [seconds] [--realtime]

The plan's bar is 24 MB/s sustained (2.4 MSPS x 5 channels x 2 bytes) to a
consumer reading with a large pipe buffer. Fast mode measures the ceiling;
--realtime checks the synthetic device keeps real time on this machine.
Run it on the machine that matters (Bill's), not only in a VM.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from atkdaq.client import open_stream  # noqa: E402


def main() -> None:
    secs = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 10.0
    realtime = "--realtime" in sys.argv
    args = ["--device", "synth", "--seconds", str(secs)]
    if not realtime:
        args += ["--set", "synth.fast=1"]
    t0 = time.perf_counter()
    nbytes = frames = gaps = 0
    last = None
    with open_stream(args) as s:
        for fr in s.frames():
            h = fr.header
            nbytes += h.hdr_bytes + h.payload_bytes
            frames += 1
            if last is not None and h.seq != last + 1:
                gaps += h.seq - last - 1
            last = h.seq
    wall = time.perf_counter() - t0
    print(f"{frames} frames, {nbytes / 1e6:.1f} MB in {wall:.2f} s wall = {nbytes / 1e6 / wall:.1f} MB/s "
          f"({secs / wall:.2f}x real time); frames lost on the pipe: {gaps}")
    print("need >= 24 MB/s for a KrakenSDR at 2.4 MSPS" + ("" if nbytes / 1e6 / wall >= 24 or realtime
                                                           else "  <-- BELOW"))


if __name__ == "__main__":
    main()
