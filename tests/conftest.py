"""Shared set-up: put python/ on the path and locate the built binaries.

Tests that need the program or the core library skip with a sentence when
they are not built, rather than failing — `build.bat` / `build.sh` builds
them, and a missing build is a different problem from a wrong answer."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

#: ATKDAQ_TEST_EXE runs the suite against another build of the same program
#: — tests/check_windows_build.sh uses it to run the Windows .exe under wine.
_env_exe = os.environ.get("ATKDAQ_TEST_EXE")
EXE = (Path(_env_exe) if _env_exe else
       next((p for p in (ROOT / "bin" / "atkdaq.exe", ROOT / "bin" / "atkdaq") if p.is_file()), None))
CORE = next((p for p in (ROOT / "bin" / "atkdaq_core.dll", ROOT / "bin" / "atkdaq_core.so")
             if p.is_file()), None)
FIXTURES = ROOT / "tests" / "fixtures"


@pytest.fixture
def exe():
    if EXE is None:
        pytest.skip("atkdaq is not built (build.bat / build.sh)")
    return EXE


@pytest.fixture
def core():
    if CORE is None:
        pytest.skip("atkdaq_core is not built (build.bat / build.sh)")
    from atkdaq import _core
    return _core
