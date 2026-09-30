"""The C and its numpy twin, on the same data, must agree.

src/sync.c vs reference.sync_estimate: the same integer lags, to the sample.
src/cal.c  vs reference.cal_estimate: weights to 1e-4, fractional delays to
1e-4 samples, residual to 1e-3 degrees — and both must return the PLANTED
truth, so agreeing on a wrong answer is not a pass.
"""

from __future__ import annotations

import zlib

import numpy as np
import pytest

from atkdaq import reference as ref

FS = 2_400_000.0
SHIFT = [0, 17, -5, 120, 3]
GAIN = [0.0, -1.0, 0.5, -0.4, 1.2]
PHASE = [0.0, 40.0, -100.0, 170.0, 20.0]
TAU = [0.0, 0.21, -0.33, 0.12, 0.44]


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_sync_c_and_twin_agree_and_find_the_planted_delays(core, seed):
    x = ref.make_array(65536, SHIFT, GAIN, PHASE, TAU, snr_db=15, seed=seed)
    rc, lag_c, ptn_c, worst_c = core.sync_estimate(x, 4096)
    tw = ref.sync_estimate(x, 4096)
    assert rc == 0
    assert list(lag_c) == SHIFT, "C missed the planted delays"
    assert list(tw.lag) == SHIFT, "twin missed the planted delays"
    assert np.allclose(ptn_c[1:], tw.ptn_db[1:], atol=0.02)
    assert worst_c > 25


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_cal_c_and_twin_agree_and_find_the_planted_response(core, seed):
    x = ref.make_array(65536, [0] * 5, GAIN, PHASE, TAU, snr_db=15, seed=seed)
    rc, c = core.cal_estimate(x, FS)
    t = ref.cal_estimate(x, FS)
    assert rc == 0 and t.ok
    # C (float32 output) and twin (float64) on identical input
    assert np.allclose(c["w"], t.w, rtol=2e-4, atol=2e-5)
    assert np.allclose(c["frac"], t.frac, atol=1e-4)
    assert np.allclose(c["resid_deg"][1:], t.resid_deg[1:], atol=1e-3)
    assert np.allclose(c["coherence"], t.coherence, atol=1e-5)
    # and both against the planted truth
    for i in range(1, 5):
        want_w = (10 ** (GAIN[0] / 20) * np.exp(1j * np.radians(PHASE[0]))) / (
            10 ** (GAIN[i] / 20) * np.exp(1j * np.radians(PHASE[i])))
        assert abs(20 * np.log10(abs(t.w[i]) / abs(want_w))) < 0.1
        assert abs(np.degrees(np.angle(t.w[i] / want_w))) < 0.5
        assert abs(t.frac[i] - (TAU[i] - TAU[0])) < 0.01
    assert t.spread_deg < 2.0


def test_cal_refuses_independent_noise_in_both(core):
    rng = np.random.default_rng(5)
    x = rng.standard_normal((5, 65536)) + 1j * rng.standard_normal((5, 65536))
    rc, c = core.cal_estimate(x, FS)
    t = ref.cal_estimate(x, FS)
    assert rc == -3 and not t.ok
    assert np.allclose(c["coherence"], t.coherence, atol=1e-5)


def test_conversion_and_crc_agree(core):
    rng = np.random.default_rng(3)
    raw = rng.integers(0, 256, 20000, dtype=np.uint8)
    raw[:10] = [0, 128, 255, 3, 128, 128, 7, 0, 255, 255]
    out_c, clip_c = core.u8_to_ci8(raw)
    out_t, clip_t = ref.u8_to_ci8(raw)
    assert np.array_equal(out_c, out_t)
    assert clip_c == clip_t
    data = rng.bytes(4097)
    assert core.crc32(data) == zlib.crc32(data) & 0xFFFFFFFF


def test_drops_twin_matches_the_c_smoke_scenario():
    """The same scenario as test_smoke.c's drop test, through the twin: a
    5000-sample loss on channel 2 is flagged on channel 2 alone and sized."""
    rng = np.random.default_rng(9)
    fs, T, P = 2.4e6, 131072, 1e9 / 2.4e6
    start = [0, 130, -40, 900, 12]
    lost = [0] * 5
    n = [0] * 5
    hist = [([], []) for _ in range(5)]
    d = ref.Drops(5, fs)
    flagged = None
    for k in range(120):
        if k == 60:
            lost[2] += 5000
        for i in range(5):
            n[i] += T
            hist[i][0].append(n[i])
            hist[i][1].append(1e9 + (n[i] + start[i] + lost[i]) * P + 5e5 + rng.uniform(0, 4e5))
        if k == 20:
            d.rebase(hist)
        if k > 20:
            m = d.update(hist)
            if m and flagged is None:
                flagged = (k, m, int(d.est_samples[2]))
    assert flagged is not None
    k, m, est = flagged
    assert m == 1 << 2
    assert 60 <= k <= 80
    assert abs(est - 5000) < 1000
