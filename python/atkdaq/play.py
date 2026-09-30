"""Replay a .atkq file from Python — the device-less test path.

    for fr in play("T1_noise.atkq", realtime=False):
        ...

With realtime=True each frame is yielded when (stream_sample0 − first)/fs of
wall clock has passed, like `atkdaq play` does on stdout; with loop=True the
file restarts at the end (stream_sample0 goes backwards, which a consumer
must treat as a new stream).
"""

from __future__ import annotations

import time
from pathlib import Path

from .frame import read_frames


def play(path, realtime: bool = False, speed: float = 1.0, loop: bool = False):
    path = Path(path)
    while True:
        t0 = time.monotonic()
        first = None
        with path.open("rb") as f:
            for fr in read_frames(f):
                h = fr.header
                if first is None:
                    first = h.stream_sample0
                if realtime and h.fs_hz and h.stream_sample0 >= first:
                    due = t0 + (h.stream_sample0 - first) / h.fs_hz / max(speed, 1e-9)
                    wait = due - time.monotonic()
                    if wait > 0:
                        time.sleep(wait)
                yield fr
        if not loop or first is None:
            return
