"""numpy twins of atkdaq's C algorithms — the specification the C must meet.

Every function here computes exactly what its C counterpart computes, from
the same inputs, by the same method, so `tests/test_cross_check.py` can feed
both the same synthetic data and require agreement:

    sync_estimate   src/sync.c   integer delays from the noise source
    cal_estimate    src/cal.c    weights + fractional delays + residual
    u8_to_ci8       src/frame.c  raw RTL bytes -> int8, clip count
    u8_to_cf32      src/frame.c  raw RTL bytes -> float
    sclock_intercept src/clock.c lower-envelope clock
    Drops           src/drops.c  per-channel loss against the common mode

and `make_array` builds the known-answer input both are tested on: a common
noise source through each receiver's H_i(f) = g·e^{jφ}·e^{-j2πfτ/fs}, with
integer stream offsets and independent receiver noise — the same model the
synthetic device (src/devices/synth.c) implements.

A change to a C algorithm is a change to its twin, in the same commit, or
the cross-check fails. That is the rule atkdsp set, carried over.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

CAL_NFFT = 1024
CAL_BAND = 0.35
CAL_MIN_COH = 0.5
SYNC_EXCLUDE = 16


def _pow2_at_least(n: int) -> int:
    m = 1
    while m < n:
        m <<= 1
    return m


# ---------------------------------------------------------------------------
# sync
# ---------------------------------------------------------------------------

@dataclass
class SyncResult:
    lag: np.ndarray            # x_i[j + lag[i]] ~ x_0[j]
    ptn_db: np.ndarray
    frac: np.ndarray
    worst_ptn_db: float


def sync_estimate(x: np.ndarray, max_lag: int) -> SyncResult:
    """x: (n_ch, n) complex. The twin of atkdaq_sync_estimate."""
    x = np.asarray(x, dtype=np.complex128)
    n_ch, n = x.shape
    lag = np.zeros(n_ch, dtype=np.int64)
    ptn = np.full(n_ch, 999.0)
    frac = np.zeros(n_ch)
    if n_ch == 1:
        return SyncResult(lag, ptn, frac, 999.0)
    m = _pow2_at_least(n + max_lag + 1)
    x0 = np.fft.fft(x[0], m)
    lags = np.arange(-max_lag, max_lag + 1)
    for ch in range(1, n_ch):
        c = np.fft.ifft(np.fft.fft(x[ch], m) * np.conj(x0))
        mag = np.abs(c[lags % m])
        k = int(np.argmax(mag))
        best_l = int(lags[k])
        best = mag[k]
        away = np.abs(lags - best_l) > SYNC_EXCLUDE
        nxt = float(mag[away].max()) if away.any() else 0.0
        lag[ch] = best_l
        ptn[ch] = 20 * np.log10(best / nxt) if nxt > 0 else 999.0
        if -max_lag < best_l < max_lag:
            ym, yp = mag[k - 1], mag[k + 1]
            den = ym - 2 * best + yp
            frac[ch] = 0.5 * (ym - yp) / den if den != 0 else 0.0
    return SyncResult(lag, ptn, frac, float(ptn[1:].min()))


# ---------------------------------------------------------------------------
# calibration
# ---------------------------------------------------------------------------

@dataclass
class CalResult:
    w: np.ndarray               # complex weights, w[0] = 1
    frac: np.ndarray            # samples
    coherence: np.ndarray
    resid_deg: np.ndarray
    power_db: np.ndarray
    spread_deg: float
    bins_used: int
    ok: bool = True
    why: str = ""


def cal_estimate(x: np.ndarray, fs: float) -> CalResult:
    """x: (n_ch, n) complex, aligned. The twin of atkdaq_cal_estimate."""
    x = np.asarray(x, dtype=np.complex128)
    n_ch, n = x.shape
    N = CAL_NFFT
    k = np.arange(N)
    win = 0.5 - 0.5 * np.cos(2 * np.pi * k / N)          # periodic Hann
    u = float(np.sum(win * win))
    starts = np.arange(0, n - N + 1, N // 2)
    nseg = len(starts)
    segs = np.stack([x[:, s:s + N] for s in starts], axis=1) * win   # (ch, seg, N)
    X = np.fft.fft(segs, axis=2)
    sxx = np.sum(np.abs(X) ** 2, axis=1)                   # (ch, N)
    sx0 = np.sum(X * np.conj(X[0])[None, :, :], axis=1)    # (ch, N)

    kmax = min(int(np.floor(CAL_BAND * N)), N // 2 - 1)
    m = np.arange(-kmax, kmax + 1)
    m = m[np.abs(m) > 2]
    kk = m % N
    used = len(m)
    p00 = float(np.sum(sxx[0, kk]))

    w = np.ones(n_ch, dtype=np.complex128)
    frac = np.zeros(n_ch)
    coh_mean = np.ones(n_ch)
    resid = np.zeros(n_ch)
    power = np.zeros(n_ch)
    power[0] = 10 * np.log10(p00 / (used * nseg * u) + 1e-30)
    # neighbouring pairs in frequency order, not across the DC gap
    pair = np.nonzero(np.diff(m) == 1)[0]
    for ch in range(1, n_ch):
        s = sx0[ch, kk]
        sii = sxx[ch, kk]
        s00 = sxx[0, kk]
        c2 = np.abs(s) ** 2 / (sii * s00 + 1e-300)
        coh_mean[ch] = float(np.mean(np.sqrt(c2)))
        pii = float(np.sum(sii))
        power[ch] = 10 * np.log10(pii / (used * nseg * u) + 1e-30)
        gmag = np.sqrt(pii / (p00 + 1e-300))
        # 1. slope from neighbours
        z = np.sum(s[pair + 1] * np.conj(s[pair]))
        dphi = float(np.angle(z))
        # 2. intercept of the de-sloped spectrum
        phi0 = float(np.angle(np.sum(s * np.exp(-1j * dphi * m))))
        # 3. weighted LS on the residual phase
        c2c = np.minimum(c2, 0.999999)
        wt = c2c / (1 - c2c)
        ang = np.angle(s * np.exp(-1j * (phi0 + dphi * m)))
        sw, swx, swy = wt.sum(), (wt * m).sum(), (wt * ang).sum()
        swxx, swxy = (wt * m * m).sum(), (wt * m * ang).sum()
        den = sw * swxx - swx * swx
        if den != 0:
            b = (sw * swxy - swx * swy) / den
            a = (swy - b * swx) / sw
        else:
            a = b = 0.0
        phase = phi0 + a
        dphi += b
        tau = -dphi * N / (2 * np.pi)
        frac[ch] = tau
        w[ch] = np.exp(-1j * phase) / gmag
        # 4. residual
        ang = np.angle(s * np.exp(-1j * (phase + dphi * m)))
        resid[ch] = np.degrees(np.sqrt(np.sum(wt * ang ** 2) / wt.sum())) if wt.sum() > 0 else 180.0
    ok = bool(np.all(coh_mean[1:] >= CAL_MIN_COH))
    return CalResult(w, frac, coh_mean, resid, power,
                     float(resid.max()) if n_ch > 1 else 0.0, used, ok,
                     "" if ok else "the noise source was not seen")


# ---------------------------------------------------------------------------
# conversions, clock, drops
# ---------------------------------------------------------------------------

def u8_to_ci8(raw: np.ndarray) -> tuple[np.ndarray, int]:
    raw = np.asarray(raw, dtype=np.uint8)
    out = (raw ^ 0x80).view(np.int8)
    pairs = raw.reshape(-1, 2)
    clip = int(np.sum(np.any((pairs == 0) | (pairs == 255), axis=1)))
    return out, clip


def u8_to_cf32(raw: np.ndarray) -> np.ndarray:
    return ((np.asarray(raw, dtype=np.float32) - 127.5) / 127.5).astype(np.float32)


def sclock_intercept(n_end: np.ndarray, t_ns: np.ndarray, fs: float,
                     last: int = 0) -> float:
    n_end = np.asarray(n_end, dtype=np.float64)
    t_ns = np.asarray(t_ns, dtype=np.float64)
    if last:
        n_end, t_ns = n_end[-last:], t_ns[-last:]
    return float(np.min(t_ns - n_end * (1e9 / fs)))


@dataclass
class Drops:
    """The twin of src/drops.c."""
    n_ch: int
    fs: float
    recent: int = 8
    thresh_ns: float = 1e6
    persist: int = 2
    ref: np.ndarray | None = None
    streak: np.ndarray = field(default_factory=lambda: np.zeros(16, dtype=int))
    est_samples: np.ndarray = field(default_factory=lambda: np.zeros(16, dtype=np.int64))
    common_samples: int = 0

    def rebase(self, clocks: list) -> None:
        self.ref = np.array([sclock_intercept(n, t, self.fs, self.recent) for n, t in clocks])
        self.streak[:] = 0
        self.est_samples[:] = 0
        self.common_samples = 0

    def update(self, clocks: list) -> int:
        if self.ref is None or self.n_ch < 2:
            return 0
        a = np.array([sclock_intercept(n, t, self.fs, self.recent) for n, t in clocks])
        shift = a - self.ref
        med = float(np.median(shift))
        period = 1e9 / self.fs
        self.common_samples = int(round(med / period))
        mask = 0
        for i in range(self.n_ch):
            dev = shift[i] - med
            self.streak[i] = self.streak[i] + 1 if abs(dev) > self.thresh_ns else 0
            self.est_samples[i] = int(round(dev / period))
            if self.streak[i] >= self.persist:
                mask |= 1 << i
        return mask


# ---------------------------------------------------------------------------
# the known-answer array
# ---------------------------------------------------------------------------

def make_array(n: int, shift, gain_db, phase_deg, tau, snr_db: float = 20.0,
               seed: int = 0, level: float = 0.05) -> np.ndarray:
    """(n_ch, n) complex128: a common noise source seen through each channel's
    receiver response, with x_i[j + shift_i] == x_0[j] (noise-free), so
    sync_estimate must return lag == shift and cal_estimate must return
    w_i = g_0 e^{jφ_0} / (g_i e^{jφ_i}) and frac_i = τ_i − τ_0."""
    rng = np.random.default_rng(seed)
    shift = np.asarray(shift, dtype=int)
    n_ch = len(shift)
    extra = int(np.max(np.abs(shift))) if n_ch else 0
    B = _pow2_at_least(n + 2 * extra + 64)
    common = (rng.standard_normal(B) + 1j * rng.standard_normal(B)) * np.sqrt(B * level / 2)
    m = np.fft.fftfreq(B) * B
    out = np.empty((n_ch, n), dtype=np.complex128)
    nvar = level * 10 ** (-snr_db / 10)
    for i in range(n_ch):
        g = 10 ** (gain_db[i] / 20)
        h = g * np.exp(1j * (np.radians(phase_deg[i]) - 2 * np.pi * m * tau[i] / B))
        xi = np.fft.ifft(common * h)
        idx = np.arange(n) + extra - shift[i]
        noise = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * np.sqrt(nvar / 2)
        out[i] = xi[idx] + noise
    return out
