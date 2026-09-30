"""The wire format: the C and the Python agree byte for byte, the committed
fixture obeys every rule in include/atkdaq_frame.h, and a damaged stream is
survived without ever returning a frame that did not validate."""

from __future__ import annotations

import ctypes as C
import io
import struct

import numpy as np
import pytest

from atkdaq import frame as F

from conftest import FIXTURES

FIXTURE = FIXTURES / "synth_240k.atkq"


class CHdr(C.Structure):
    _fields_ = [("magic", C.c_char * 4), ("version", C.c_uint16), ("hdr_bytes", C.c_uint16),
                ("ch_offset", C.c_uint16), ("ch_stride", C.c_uint16), ("n_channels", C.c_uint8),
                ("fmt", C.c_uint8), ("cal_state", C.c_uint8), ("device_kind", C.c_uint8),
                ("flags", C.c_uint32), ("seq", C.c_uint32), ("t_utc_ns", C.c_int64),
                ("fc_hz", C.c_uint64), ("fs_hz", C.c_uint32), ("samples_per_ch", C.c_uint32),
                ("segment", C.c_uint32), ("passport", C.c_uint32), ("cal_age_ms", C.c_uint32),
                ("spread_cdeg", C.c_uint16), ("sync_conf", C.c_uint16),
                ("gain_tenths_db", C.c_int16), ("mode", C.c_uint16), ("payload_bytes", C.c_uint32),
                ("stream_sample0", C.c_uint64), ("seg_sample0", C.c_uint64),
                ("hdr_crc32", C.c_uint32), ("reserved", C.c_uint32)]


class CCh(C.Structure):
    _fields_ = [("delay", C.c_int32), ("frac_delay", C.c_float), ("w_re", C.c_float),
                ("w_im", C.c_float), ("drops", C.c_uint32), ("clip", C.c_uint32),
                ("sample_count", C.c_uint64)]


def test_the_documented_offsets_hold_in_python():
    assert F.FIXED.size == 96 and F.CHREC.size == 32
    # the offsets atkdaq_frame.h documents, field by field
    assert C.sizeof(CHdr) == 96 and C.sizeof(CCh) == 32
    for name, off in [("t_utc_ns", 24), ("fc_hz", 32), ("fs_hz", 40), ("spread_cdeg", 60),
                      ("gain_tenths_db", 64), ("payload_bytes", 68), ("stream_sample0", 72),
                      ("seg_sample0", 80), ("hdr_crc32", 88)]:
        assert getattr(CHdr, name).offset == off, name
    assert CCh.sample_count.offset == 24


def test_c_packs_python_parses_and_back(core):
    lib = core.load()
    lib.atkdaq_hdr_pack.argtypes = [C.c_void_p, C.c_size_t, C.POINTER(CHdr), C.POINTER(CCh), C.c_int]
    lib.atkdaq_hdr_pack.restype = C.c_size_t
    lib.atkdaq_hdr_unpack.argtypes = [C.c_void_p, C.c_size_t, C.POINTER(CHdr), C.POINTER(CCh), C.c_int]
    lib.atkdaq_hdr_unpack.restype = C.c_int
    h = CHdr(fmt=0, cal_state=2, device_kind=2, flags=F.F_NOISE_ON | F.F_SYNTHETIC, seq=77,
             t_utc_ns=1790000000123456789, fc_hz=446_000_000, fs_hz=2_400_000,
             samples_per_ch=120000, segment=3, passport=9, cal_age_ms=1234, spread_cdeg=57,
             sync_conf=3812, gain_tenths_db=280, mode=1, stream_sample0=987654321, seg_sample0=11)
    chs = (CCh * 5)()
    for i in range(5):
        chs[i] = CCh(delay=i * 7 - 3, frac_delay=0.125 * i, w_re=1 - 0.1 * i, w_im=0.05 * i,
                     drops=i, clip=10 * i, sample_count=10 ** 6 * i)
    buf = C.create_string_buffer(512)
    n = lib.atkdaq_hdr_pack(buf, 512, C.byref(h), chs, 5)
    assert n == 256
    ph = F.parse_header(buf.raw[:n])
    assert (ph.seq, ph.t_utc_ns, ph.fc_hz, ph.stream_sample0, ph.flags, ph.gain_tenths_db) == \
        (77, 1790000000123456789, 446_000_000, 987654321, F.F_NOISE_ON | F.F_SYNTHETIC, 280)
    assert [c.delay for c in ph.channels] == [i * 7 - 3 for i in range(5)]
    assert ph.channels[4].frac_delay == pytest.approx(0.5)
    # Python packs, C unpacks
    raw = F.pack_header(ph)
    assert raw == buf.raw[:n], "the Python and C serialisations differ"
    h2 = CHdr()
    ch2 = (CCh * 5)()
    assert lib.atkdaq_hdr_unpack(raw, len(raw), C.byref(h2), ch2, 5) == 256


def test_the_committed_fixture_obeys_the_header_rules():
    if not FIXTURE.is_file():
        pytest.skip("fixture not present")
    with FIXTURE.open("rb") as f:
        frames = list(F.read_frames(f))
    assert F.read_frames.skipped == 0
    assert len(frames) == 20
    hs = [fr.header for fr in frames]
    assert all(h.has(F.F_SYNTHETIC) for h in hs)
    assert hs[0].has(F.F_SEGMENT_START)
    for a, b in zip(hs, hs[1:]):
        assert b.seq == a.seq + 1
        assert b.stream_sample0 == a.stream_sample0 + a.samples_per_ch
        assert b.payload_bytes == b.n_channels * b.samples_per_ch * 2
    usable = [fr for fr in frames if fr.usable_for_df]
    assert usable
    h = usable[-1].header
    assert list(h.delays) == [0, -17, 5, -120, -3]
    assert np.allclose(h.frac_delays, [0, 0.21, -0.33, 0.12, 0.44], atol=0.02)
    # a delay change is a discontinuity, and the passport identifies it
    first_cal = next(i for i, x in enumerate(hs) if x.passport == 1)
    assert hs[first_cal].has(F.F_DISCONTINUITY)


def test_the_fixture_emitter_comes_back_with_its_steering_phases():
    if not FIXTURE.is_file():
        pytest.skip("fixture not present")
    with FIXTURE.open("rb") as f:
        frames = [fr for fr in F.read_frames(f) if fr.usable_for_df]
    fr = frames[-1]
    y = fr.calibrated()
    Y = np.fft.fft(y, axis=1)
    f = np.fft.fftfreq(y.shape[1], 1 / fr.header.fs_hz)
    band = np.abs(f - 30000) < 5000
    rel = np.array([np.degrees(np.angle(np.sum(Y[i, band] * np.conj(Y[0, band]))))
                    for i in range(5)])
    err = (rel - np.array([0, 72, 144, 216, 288]) + 180) % 360 - 180
    assert np.all(np.abs(err) < 2.0), rel


def test_a_damaged_stream_is_resynchronised_never_misread():
    if not FIXTURE.is_file():
        pytest.skip("fixture not present")
    data = bytearray(FIXTURE.read_bytes())
    fb = len(data) // 20
    data[fb * 3 + 5] ^= 0xFF                   # frame 3's header: CRC fails
    data[fb * 7:fb * 7] = b"ATKQ garbage \x0a\x0d" * 20     # junk inserted before frame 7
    frames = list(F.read_frames(io.BytesIO(bytes(data))))
    assert len(frames) == 19
    assert F.read_frames.skipped > 0
    seqs = [fr.header.seq for fr in frames]
    assert 3 not in seqs and seqs == sorted(seqs)


def test_a_newer_wire_format_is_refused_by_name():
    if not FIXTURE.is_file():
        pytest.skip("fixture not present")
    data = bytearray(FIXTURE.read_bytes()[:256])
    struct.pack_into("<H", data, 4, 2)
    struct.pack_into("<I", data, 88, F.crc32_of_header(data, 256))   # a VALID v2 header
    with pytest.raises(F.FrameError) as e:
        F.parse_header(bytes(data))
    assert e.value.code == -3 and "version 2" in str(e.value)
