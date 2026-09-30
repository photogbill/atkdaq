"""ctypes binding to atkdaq_core (bin/atkdaq_core.dll or .so) — for the
cross-check tests only. atkdaq.exe links the same sources statically; this
library exists so tests can call the C with the same arrays the numpy twins
get. Nothing in ATK or atkdf uses it."""

from __future__ import annotations

import ctypes as C
import os
import sys
from pathlib import Path

import numpy as np

ABI = 1
MAX_CH = 16

_lib = None


def lib_path() -> Path:
    root = Path(__file__).resolve().parents[2]
    name = "atkdaq_core.dll" if sys.platform == "win32" else "atkdaq_core.so"
    env = os.environ.get("ATKDAQ_CORE")
    return Path(env) if env else root / "bin" / name


class SyncResult(C.Structure):
    _fields_ = [("lag", C.c_int32 * MAX_CH), ("ptn_db", C.c_float * MAX_CH),
                ("frac", C.c_float * MAX_CH), ("worst_ptn_db", C.c_float)]


class CalResult(C.Structure):
    _fields_ = [("w_re", C.c_float * MAX_CH), ("w_im", C.c_float * MAX_CH),
                ("frac", C.c_float * MAX_CH), ("coherence", C.c_float * MAX_CH),
                ("resid_deg", C.c_float * MAX_CH), ("power_db", C.c_float * MAX_CH),
                ("spread_deg", C.c_float), ("bins_used", C.c_int)]


def load():
    global _lib
    if _lib is not None:
        return _lib
    p = lib_path()
    if not p.is_file():
        raise FileNotFoundError(f"{p} not built - run build.bat / build.sh")
    lib = C.CDLL(str(p))
    lib.atkdaq_core_abi.restype = C.c_int
    if lib.atkdaq_core_abi() != ABI:
        raise ImportError(f"atkdaq_core ABI {lib.atkdaq_core_abi()}, binding expects {ABI}")
    fp = C.POINTER(C.c_float)
    lib.atkdaq_sync_estimate.argtypes = [C.POINTER(fp), C.c_int, C.c_int, C.c_int,
                                         C.POINTER(SyncResult)]
    lib.atkdaq_sync_estimate.restype = C.c_int
    lib.atkdaq_cal_estimate.argtypes = [C.POINTER(fp), C.c_int, C.c_int, C.c_double,
                                        C.POINTER(CalResult)]
    lib.atkdaq_cal_estimate.restype = C.c_int
    lib.atkdaq_u8_to_ci8.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t]
    lib.atkdaq_u8_to_ci8.restype = C.c_uint32
    lib.atkdaq_crc32.argtypes = [C.c_uint32, C.c_void_p, C.c_size_t]
    lib.atkdaq_crc32.restype = C.c_uint32
    _lib = lib
    return lib


def _ptrs(x: np.ndarray):
    """(n_ch, n) complex -> keep-alive list of float32 interleaved arrays and
    a float** for C."""
    arrs = [np.ascontiguousarray(np.stack([row.real, row.imag], axis=1).astype(np.float32).ravel())
            for row in np.asarray(x)]
    fp = C.POINTER(C.c_float)
    table = (fp * len(arrs))(*[a.ctypes.data_as(fp) for a in arrs])
    return arrs, table


def sync_estimate(x: np.ndarray, max_lag: int):
    lib = load()
    arrs, table = _ptrs(x)
    r = SyncResult()
    rc = lib.atkdaq_sync_estimate(table, x.shape[0], x.shape[1], max_lag, C.byref(r))
    n = x.shape[0]
    return rc, np.array(r.lag[:n]), np.array(r.ptn_db[:n]), float(r.worst_ptn_db)


def cal_estimate(x: np.ndarray, fs: float):
    lib = load()
    arrs, table = _ptrs(x)
    r = CalResult()
    rc = lib.atkdaq_cal_estimate(table, x.shape[0], x.shape[1], fs, C.byref(r))
    n = x.shape[0]
    w = np.array(r.w_re[:n]) + 1j * np.array(r.w_im[:n])
    return rc, dict(w=w, frac=np.array(r.frac[:n]), coherence=np.array(r.coherence[:n]),
                    resid_deg=np.array(r.resid_deg[:n]), power_db=np.array(r.power_db[:n]),
                    spread_deg=float(r.spread_deg), bins_used=int(r.bins_used))


def u8_to_ci8(raw: np.ndarray):
    lib = load()
    raw = np.ascontiguousarray(raw, dtype=np.uint8)
    out = np.empty(raw.size, dtype=np.int8)
    clip = lib.atkdaq_u8_to_ci8(out.ctypes.data, raw.ctypes.data, raw.size)
    return out, int(clip)


def crc32(data: bytes) -> int:
    lib = load()
    buf = C.create_string_buffer(data, len(data))
    return int(lib.atkdaq_crc32(0, buf, len(data)))
