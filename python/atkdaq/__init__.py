"""atkdaq — the Python side of the coherent multi-channel DAQ.

    frame.py      the wire format (include/atkdaq_frame.h): parse, pack, read a stream
    reference.py  numpy twins of the C algorithms (sync, cal, clock, drops)
    _core.py      ctypes binding to bin/atkdaq_core, for the cross-check tests
    client.py     start atkdaq.exe, iterate its frames, send it commands
    play.py       replay a .atkq file from Python, paced or not

Fixtures are recorded by the program itself (atkdaq record --device synth),
so a fixture is always what the real writer wrote; frame.make_frame builds
single frames for the tests that need a specific header.

LICENCE. This package is part of atkdaq and GPL-2.0 like the program (which
links librtlsdr). ATK and atkdf deliberately do NOT import it: they talk to
atkdaq.exe over a pipe and read the documented wire format with their own
reader (atkdf.atkq), which keeps both of them free of the GPL — the same
arm's-length arrangement ATK already has with rtl_sdr.exe and dsd-neo.
"""

from .frame import (CAL_CALIBRATED, CAL_NAMES, F_DISCONTINUITY, F_MISALIGNED,
                    F_NOISE_ON, F_RETUNE, F_SEGMENT_START, F_SYNTHETIC, Frame,
                    FrameError, Header, parse_header, read_frames)

__version__ = "0.1.0"
WIRE_VERSION = 1

__all__ = ["Frame", "FrameError", "Header", "parse_header", "read_frames",
           "CAL_CALIBRATED", "CAL_NAMES", "F_DISCONTINUITY", "F_MISALIGNED",
           "F_NOISE_ON", "F_RETUNE", "F_SEGMENT_START", "F_SYNTHETIC",
           "open_stream", "find_exe"]


def open_stream(*args, **kw):
    from .client import open_stream as _o
    return _o(*args, **kw)


def find_exe(*args, **kw):
    from .client import find_exe as _f
    return _f(*args, **kw)
