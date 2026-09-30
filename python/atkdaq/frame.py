"""The atkdaq wire format in Python — `include/atkdaq_frame.h`, field for field.

`tests/test_frames.py` holds this file and the C header together: the C
smoke test packs headers, this module parses them, and the offsets below are
asserted against the documented layout. If this file and the header ever
disagree, the header wins and this file is the bug.

Reading a stream::

    from atkdaq.frame import read_frames
    with open("T1.atkq", "rb") as f:
        for fr in read_frames(f):
            if fr.usable_for_df:
                x = fr.calibrated()          # (n_ch, N) complex64, weights AND slope applied

`Frame.iq` is the raw aligned payload as complex64 (delays already applied by
the DAQ, weights NOT). `Frame.calibrated()` applies the calibration the way
the header says it must be applied — the complex weight AND the fractional-
delay phase slope across the band — and `Frame.calibration_at(f0)` gives the
per-channel factor for a narrowband signal at baseband offset f0.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field
from typing import BinaryIO, Iterator

import numpy as np

MAGIC = b"ATKQ"
VERSION = 1
FIXED_BYTES = 96
CH_STRIDE = 32
MAX_CHANNELS = 16

FMT_CI8 = 0

F_MISALIGNED = 1 << 0
F_NOISE_ON = 1 << 1
F_SEGMENT_START = 1 << 2
F_CAL_STALE = 1 << 3
F_OVERRUN = 1 << 4
F_RETUNE = 1 << 5
F_CLIPPED = 1 << 6
F_DISCONTINUITY = 1 << 7
F_SYNTHETIC = 1 << 8

FLAG_NAMES = {F_MISALIGNED: "MISALIGNED", F_NOISE_ON: "NOISE_ON",
              F_SEGMENT_START: "SEGMENT_START", F_CAL_STALE: "CAL_STALE",
              F_OVERRUN: "OVERRUN", F_RETUNE: "RETUNE", F_CLIPPED: "CLIPPED",
              F_DISCONTINUITY: "DISCONTINUITY", F_SYNTHETIC: "SYNTHETIC"}

CAL_NONE, CAL_SYNCED, CAL_CALIBRATED, CAL_MISALIGNED, CAL_CALIBRATING = range(5)
CAL_NAMES = {0: "none", 1: "synced", 2: "calibrated", 3: "misaligned",
             4: "calibrating"}

DEV_KRAKEN, DEV_SYNTH = 1, 2
MODE_STATIC, MODE_MOBILE = 0, 1

# The fixed part, little-endian. Offsets (checked in tests/test_frames.py):
#   0 magic 4s | 4 version H | 6 hdr_bytes H | 8 ch_offset H | 10 ch_stride H
#  12 n_channels B | 13 fmt B | 14 cal_state B | 15 device_kind B
#  16 flags I | 20 seq I | 24 t_utc_ns q | 32 fc_hz Q | 40 fs_hz I
#  44 samples_per_ch I | 48 segment I | 52 passport I | 56 cal_age_ms I
#  60 spread_cdeg H | 62 sync_conf H | 64 gain_tenths_db h | 66 mode H
#  68 payload_bytes I | 72 stream_sample0 Q | 80 seg_sample0 Q
#  88 hdr_crc32 I | 92 reserved I
FIXED = struct.Struct("<4sHHHHBBBBIIqQIIIIIHHhHIQQII")
CHREC = struct.Struct("<ifffIIQ")   # delay frac_delay w_re w_im drops clip sample_count
assert FIXED.size == FIXED_BYTES
assert CHREC.size == CH_STRIDE


class FrameError(ValueError):
    """A header that does not validate. `code` matches atkdaq_hdr_unpack."""

    def __init__(self, code: int, msg: str):
        super().__init__(msg)
        self.code = code


@dataclass
class Channel:
    delay: int
    frac_delay: float
    w: complex
    drops: int
    clip: int
    sample_count: int


@dataclass
class Header:
    version: int
    hdr_bytes: int
    ch_offset: int
    ch_stride: int
    n_channels: int
    fmt: int
    cal_state: int
    device_kind: int
    flags: int
    seq: int
    t_utc_ns: int
    fc_hz: int
    fs_hz: int
    samples_per_ch: int
    segment: int
    passport: int
    cal_age_ms: int
    spread_cdeg: int
    sync_conf: int
    gain_tenths_db: int
    mode: int
    payload_bytes: int
    stream_sample0: int
    seg_sample0: int
    hdr_crc32: int
    channels: list = field(default_factory=list)

    # -- convenience --------------------------------------------------------
    def has(self, flag: int) -> bool:
        return bool(self.flags & flag)

    @property
    def flag_names(self) -> list:
        return [n for f, n in FLAG_NAMES.items() if self.flags & f]

    @property
    def spread_deg(self) -> float | None:
        return None if self.spread_cdeg == 0xFFFF else self.spread_cdeg / 100.0

    @property
    def sync_ptn_db(self) -> float:
        return self.sync_conf / 100.0

    @property
    def t_utc(self) -> float:
        return self.t_utc_ns / 1e9

    @property
    def delays(self) -> np.ndarray:
        return np.array([c.delay for c in self.channels], dtype=np.int64)

    @property
    def weights(self) -> np.ndarray:
        return np.array([c.w for c in self.channels], dtype=np.complex128)

    @property
    def frac_delays(self) -> np.ndarray:
        return np.array([c.frac_delay for c in self.channels], dtype=np.float64)

    @property
    def usable_for_df(self) -> bool:
        """Calibrated, aligned, antennas connected, tuning settled — the one
        test every direction-finding consumer must apply to every frame."""
        bad = F_MISALIGNED | F_NOISE_ON | F_RETUNE | F_CAL_STALE
        return self.cal_state == CAL_CALIBRATED and not (self.flags & bad)

    def describe(self) -> str:
        return (f"seq {self.seq} seg {self.segment} passport {self.passport} "
                f"{CAL_NAMES.get(self.cal_state, '?')} fc {self.fc_hz / 1e6:.4f} MHz "
                f"s0 {self.stream_sample0} [{' '.join(self.flag_names) or '-'}]")


def crc32_of_header(buf: bytes | memoryview, hdr_bytes: int) -> int:
    """CRC-32 of the header with the CRC field (bytes 88..91) zeroed."""
    b = bytes(buf[:hdr_bytes])
    return zlib.crc32(b[:88] + b"\0\0\0\0" + b[92:]) & 0xFFFFFFFF


def parse_header(buf: bytes | memoryview) -> Header:
    """Validate and parse. Raises FrameError with atkdaq_hdr_unpack's codes:
    -1 need more bytes, -2 bad magic, -3 version, -4 CRC, -5 sizes."""
    if len(buf) < FIXED_BYTES:
        raise FrameError(-1, "need more bytes")
    f = FIXED.unpack_from(buf, 0)
    (magic, version, hdr_bytes, ch_off, ch_stride, n, fmt, cal_state, dev,
     flags, seq, t_utc, fc, fs, spc, seg, passport, age, spread, conf, gain,
     mode, payload, s0, segs0, crc, _res) = f
    if magic != MAGIC:
        raise FrameError(-2, f"bad magic {magic!r}")
    # SIZES AND CRC BEFORE THE VERSION: a flipped bit in the version field is
    # damage to skip, not a newer format to refuse, and only the CRC can tell
    # the two apart — so an unknown version is believed only when the header
    # checks.
    if (not 1 <= n <= MAX_CHANNELS or ch_off < FIXED_BYTES
            or ch_stride < CH_STRIDE or hdr_bytes < ch_off + n * ch_stride):
        raise FrameError(-5, "inconsistent header sizes")
    if len(buf) < hdr_bytes:
        raise FrameError(-1, "need more bytes")
    if crc32_of_header(buf, hdr_bytes) != crc:
        raise FrameError(-4, "header CRC mismatch")
    if version != VERSION:
        raise FrameError(-3, f"wire format version {version}; this reader "
                             f"understands version {VERSION}")
    if payload != n * spc * 2:
        raise FrameError(-5, "payload size does not match n_channels x samples_per_ch x 2")
    chans = []
    for i in range(n):
        d, fr, wr, wi, drops, clip, count = CHREC.unpack_from(buf, ch_off + i * ch_stride)
        chans.append(Channel(d, fr, complex(wr, wi), drops, clip, count))
    return Header(version, hdr_bytes, ch_off, ch_stride, n, fmt, cal_state, dev,
                  flags, seq, t_utc, fc, fs, spc, seg, passport, age, spread,
                  conf, gain, mode, payload, s0, segs0, crc, chans)


def pack_header(h: Header) -> bytes:
    """Serialise (for fixtures and tests); computes sizes and the CRC."""
    n = len(h.channels)
    hdr_bytes = FIXED_BYTES + n * CH_STRIDE
    payload = n * h.samples_per_ch * 2
    buf = bytearray(hdr_bytes)
    FIXED.pack_into(buf, 0, MAGIC, VERSION, hdr_bytes, FIXED_BYTES, CH_STRIDE, n,
                    h.fmt, h.cal_state, h.device_kind, h.flags, h.seq & 0xFFFFFFFF,
                    h.t_utc_ns, h.fc_hz, h.fs_hz, h.samples_per_ch, h.segment,
                    h.passport, h.cal_age_ms, h.spread_cdeg, h.sync_conf,
                    h.gain_tenths_db, h.mode, payload, h.stream_sample0,
                    h.seg_sample0, 0, 0)
    for i, c in enumerate(h.channels):
        CHREC.pack_into(buf, FIXED_BYTES + i * CH_STRIDE, int(c.delay),
                        float(c.frac_delay), float(complex(c.w).real),
                        float(complex(c.w).imag), int(c.drops), int(c.clip),
                        int(c.sample_count))
    crc = zlib.crc32(bytes(buf)) & 0xFFFFFFFF
    struct.pack_into("<I", buf, 88, crc)
    return bytes(buf)


class Frame:
    """One frame: a validated header and a zero-copy view of its payload."""

    __slots__ = ("header", "payload")

    def __init__(self, header: Header, payload: bytes | memoryview):
        self.header = header
        self.payload = payload

    # -- raw ----------------------------------------------------------------
    @property
    def ci8(self) -> np.ndarray:
        """(n_ch, N, 2) int8 view of the payload — no copy."""
        h = self.header
        return np.frombuffer(self.payload, dtype=np.int8).reshape(
            h.n_channels, h.samples_per_ch, 2)

    @property
    def iq(self) -> np.ndarray:
        """(n_ch, N) complex64, aligned, NOT calibrated. Scale: (v + 0.5)/128,
        so the RTL's half-LSB offset does not become a DC spike."""
        a = self.ci8.astype(np.float32)
        a += 0.5
        a *= 1.0 / 128.0
        return a[..., 0] + 1j * a[..., 1]

    # -- calibration ----------------------------------------------------------
    def calibration_at(self, f0_hz: float) -> np.ndarray:
        """Per-channel complex factor for a narrowband signal at baseband
        offset f0: w[i]·exp(+j·2π·f0·frac[i]/fs). Multiply channel i by it."""
        h = self.header
        return (h.weights * np.exp(2j * np.pi * f0_hz * h.frac_delays / h.fs_hz)
                ).astype(np.complex64)

    def calibrated(self) -> np.ndarray:
        """(n_ch, N) complex64 with the weights AND the fractional-delay slope
        applied across the whole band (FFT domain; circular at the frame
        edges, which a consumer processing bursts inside a frame ignores)."""
        h = self.header
        x = self.iq
        n = x.shape[1]
        f = np.fft.fftfreq(n, d=1.0 / h.fs_hz)
        ramp = np.exp(2j * np.pi * np.outer(h.frac_delays, f) / h.fs_hz)
        y = np.fft.ifft(np.fft.fft(x, axis=1) * ramp, axis=1)
        return (y * h.weights[:, None]).astype(np.complex64)

    @property
    def usable_for_df(self) -> bool:
        return self.header.usable_for_df

    def to_bytes(self) -> bytes:
        return pack_header(self.header) + bytes(self.payload)


def make_frame(header: Header, ci8: np.ndarray) -> Frame:
    """Build a frame from a header and an (n_ch, N, 2) int8 array."""
    a = np.ascontiguousarray(ci8, dtype=np.int8)
    header.samples_per_ch = a.shape[1]
    header.payload_bytes = a.size
    raw = pack_header(header)
    header = parse_header(raw)          # sizes and CRC as they will be on the wire
    return Frame(header, a.tobytes())


def read_frames(stream: BinaryIO, resync: bool = True) -> Iterator[Frame]:
    """Frames from a binary stream (pipe or file). On a header that does not
    validate, searches forward for the next "ATKQ" whose CRC checks (and
    counts the bytes skipped in `read_frames.skipped`), or raises FrameError
    when resync is off. A version mismatch always raises: a newer stream is
    not garbage to skip, it is a reader to update."""
    buf = bytearray()
    read_frames.skipped = 0

    def fill(n: int) -> bool:
        while len(buf) < n:
            chunk = stream.read(max(n - len(buf), 1 << 20))
            if not chunk:
                return False
            buf.extend(chunk)
        return True

    while True:
        if not fill(FIXED_BYTES):
            return
        try:
            hb = struct.unpack_from("<H", buf, 6)[0]
            if buf[:4] == MAGIC and FIXED_BYTES <= hb <= FIXED_BYTES + MAX_CHANNELS * CH_STRIDE:
                if not fill(hb):
                    return
            else:
                hb = FIXED_BYTES
            # a COPY of the header bytes: a memoryview would pin `buf` (the
            # traceback of a failed parse keeps it alive) and the resync below
            # could then not shrink it
            h = parse_header(bytes(buf[:max(hb, FIXED_BYTES)]))
        except FrameError as e:
            if e.code == -3 or not resync:
                raise
            nxt = buf.find(MAGIC, 1)
            drop = nxt if nxt > 0 else max(1, len(buf) - 3)
            read_frames.skipped += drop
            del buf[:drop]
            continue
        total = h.hdr_bytes + h.payload_bytes
        if not fill(total):
            return                       # a truncated final frame is not returned
        payload = bytes(buf[h.hdr_bytes:total])
        del buf[:total]
        yield Frame(h, payload)


read_frames.skipped = 0
