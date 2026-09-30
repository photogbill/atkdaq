"""Run atkdaq.exe, read its frames, send it commands, read its status.

    from atkdaq.client import open_stream
    with open_stream(["--device", "synth", "--set", "synth.fast=1"]) as daq:
        for fr in daq.frames():
            if fr.usable_for_df:
                ...
            if enough:
                daq.control.tune(446.0e6)

`Stream.events` collects every "ATKDAQ event=..." status line as a dict
(parsed by `parse_status`); `Stream.last(event)` is the newest of a kind.
The subprocess gets its own stdin (commands), stdout (frames) and stderr
(status); closing the stream sends `quit` and waits.
"""

from __future__ import annotations

import os
import re
import shlex
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

from .frame import read_frames

_KV = re.compile(r'(\w+)=("([^"]*)"|\S+)')


def parse_status(line: str) -> dict | None:
    """'ATKDAQ event=sync passport=1 delays=0,3 msg="..."' -> dict, or None."""
    if not line.startswith("ATKDAQ "):
        return None
    out = {}
    for m in _KV.finditer(line[7:]):
        key, raw, quoted = m.group(1), m.group(2), m.group(3)
        out[key] = quoted if quoted is not None else raw
    return out


def find_exe() -> Path:
    """$ATKDAQ_EXE, then this checkout's bin/, then PATH."""
    env = os.environ.get("ATKDAQ_EXE")
    if env and Path(env).is_file():
        return Path(env)
    root = Path(__file__).resolve().parents[2]
    for name in ("atkdaq.exe", "atkdaq"):
        p = root / "bin" / name
        if p.is_file():
            return p
    w = shutil.which("atkdaq")
    if w:
        return Path(w)
    raise FileNotFoundError("atkdaq not built: run build.bat (Windows) or build.sh")


class Control:
    """The stdin command vocabulary of src/control.h."""

    def __init__(self, proc: subprocess.Popen):
        self._p = proc
        self._lock = threading.Lock()

    def send(self, line: str) -> None:
        with self._lock:
            if self._p.stdin and not self._p.stdin.closed:
                self._p.stdin.write((line.strip() + "\n").encode("ascii"))
                self._p.stdin.flush()

    def tune(self, hz: float) -> None: self.send(f"tune {int(round(hz))}")
    def gain(self, tenths_db: int) -> None: self.send(f"gain {int(tenths_db)}")
    def sync(self) -> None: self.send("sync")
    def cal(self) -> None: self.send("cal")
    def check(self) -> None: self.send("check")
    def noise(self, state: str) -> None: self.send(f"noise {state}")
    def mode(self, m: str) -> None: self.send(f"mode {m}")
    def quiet(self) -> None: self.send("quiet")
    def interval(self, what: str, seconds: int) -> None: self.send(f"interval {what} {int(seconds)}")
    def status(self) -> None: self.send("status")
    def quit(self) -> None: self.send("quit")


class Stream:
    def __init__(self, argv: list, exe: Path | None = None, env=None):
        self.exe = Path(exe) if exe else find_exe()
        self.argv = [str(self.exe)] + [str(a) for a in argv]
        kw = {}
        if sys.platform == "win32":
            kw["creationflags"] = 0x08000000          # CREATE_NO_WINDOW
        self.proc = subprocess.Popen(self.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, bufsize=0, env=env, **kw)
        self.control = Control(self.proc)
        self.events: list = []
        self._cv = threading.Condition()
        self._t = threading.Thread(target=self._read_stderr, daemon=True)
        self._t.start()

    def _read_stderr(self) -> None:
        for raw in iter(self.proc.stderr.readline, b""):
            ev = parse_status(raw.decode("utf-8", "replace").rstrip())
            if ev is not None:
                with self._cv:
                    self.events.append(ev)
                    self._cv.notify_all()

    def frames(self):
        return read_frames(self.proc.stdout)

    def last(self, event: str) -> dict | None:
        with self._cv:
            for ev in reversed(self.events):
                if ev.get("event") == event:
                    return ev
        return None

    def wait_event(self, event: str, timeout: float = 30.0, after: int = 0) -> dict | None:
        """The first `event` whose index in self.events is >= after."""
        end = time.monotonic() + timeout
        with self._cv:
            while True:
                for ev in self.events[after:]:
                    if ev.get("event") == event:
                        return ev
                left = end - time.monotonic()
                if left <= 0:
                    return None
                self._cv.wait(left)

    def close(self, timeout: float = 10.0) -> int:
        try:
            self.control.quit()
            if self.proc.stdin:
                self.proc.stdin.close()
        except OSError:
            pass
        try:
            # drain so a writer blocked on a full pipe can see the quit
            if self.proc.stdout:
                self.proc.stdout.close()
        except OSError:
            pass
        try:
            return self.proc.wait(timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            return self.proc.wait(5)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __repr__(self) -> str:
        return f"<atkdaq.Stream {' '.join(shlex.quote(a) for a in self.argv)}>"


def open_stream(argv: list | None = None, exe: Path | None = None, env=None) -> Stream:
    return Stream(list(argv or []), exe, env)
