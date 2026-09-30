/*
 * cal.c — complex weights and fractional delays from the noise source.
 *
 * After sync the channels are aligned to the nearest sample. What is left,
 * per channel relative to channel 0, is a complex gain g (the tuner PLL's
 * phase and the chain's gain, both new after every retune or gain change)
 * and a residual delay of a fraction of a sample (the RTL2832U decimators'
 * arbitrary start phases). In the frequency domain that is
 *
 *     S_i0(f) = E[ X_i(f) X_0*(f) ] = g_i · |X_0(f)|^2 · exp(-j·2π·f·τ_i/fs)
 *
 * — a phase that is a straight line in f: intercept arg(g_i), slope set by
 * τ_i. So the calibration is a weighted straight-line fit to the phase of
 * the averaged cross-spectrum, and its residual is the quality number.
 *
 * FITTING WITHOUT UNWRAPPING. The phase of S_i0 can wrap across the band, and
 * unwrapping a noisy phase is where fits like this go wrong. Instead the
 * slope is estimated first from the product of each bin with its neighbour's
 * conjugate — the phase INCREMENT per bin, which is small and never wraps —
 * the line is removed, and the fit is refined on a residual that is a few
 * degrees wide. The same scheme is in python/atkdaq/reference.py.
 *
 * WHICH BINS. Only |f| <= ATKDAQ_CAL_BAND·fs (the RTL2832U's decimating
 * filter is flat there and aliased beyond it), and never the five bins
 * nearest DC, |m| <= 2 (the LO leakage spike is correlated across channels through the
 * shared clock and is not the noise source). Each bin is weighted by the
 * Fisher information of its phase, coh^2 / (1 - coh^2).
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "atkdaq_core.h"
#include "pocketfft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NFFT ATKDAQ_CAL_NFFT

int atkdaq_cal_estimate(const float *const *iq, int n_ch, int n, double fs,
                        atkdaq_cal_result *out)
{
	double *win = NULL, *buf = NULL, *sxx = NULL, *sx0 = NULL;
	double *x0 = NULL;
	cfft_plan plan = NULL;
	int ch, seg, nseg, k, rc = 0, kmax, used = 0;
	double u = 0.0;

	memset(out, 0, sizeof(*out));
	if (n_ch < 1 || n_ch > ATKDAQ_MAX_CHANNELS || n < 4 * NFFT || fs <= 0)
		return -1;
	out->w_re[0] = 1.0f;
	out->coherence[0] = 1.0f;

	win = (double *)malloc(NFFT * sizeof(double));
	buf = (double *)malloc(2 * NFFT * sizeof(double));
	x0 = (double *)malloc(2 * NFFT * sizeof(double));
	/* per channel: auto-spectrum S_ii and cross-spectrum S_i0 (complex) */
	sxx = (double *)calloc((size_t)n_ch * NFFT, sizeof(double));
	sx0 = (double *)calloc((size_t)n_ch * 2 * NFFT, sizeof(double));
	plan = make_cfft_plan(NFFT);
	if (!win || !buf || !x0 || !sxx || !sx0 || !plan) {
		rc = -2;
		goto done;
	}
	for (k = 0; k < NFFT; k++) {
		win[k] = 0.5 - 0.5 * cos(2.0 * M_PI * k / NFFT);   /* periodic Hann */
		u += win[k] * win[k];
	}

	/* Welch, 50 % overlap */
	nseg = 0;
	for (seg = 0; seg + NFFT <= n; seg += NFFT / 2) {
		for (k = 0; k < NFFT; k++) {
			x0[2 * k] = iq[0][2 * (seg + k)] * win[k];
			x0[2 * k + 1] = iq[0][2 * (seg + k) + 1] * win[k];
		}
		if (cfft_forward(plan, x0, 1.0) != 0) {
			rc = -2;
			goto done;
		}
		for (k = 0; k < NFFT; k++)
			sxx[k] += x0[2 * k] * x0[2 * k] + x0[2 * k + 1] * x0[2 * k + 1];
		for (ch = 1; ch < n_ch; ch++) {
			double *sxxc = sxx + (size_t)ch * NFFT;
			double *sx0c = sx0 + (size_t)ch * 2 * NFFT;
			for (k = 0; k < NFFT; k++) {
				buf[2 * k] = iq[ch][2 * (seg + k)] * win[k];
				buf[2 * k + 1] = iq[ch][2 * (seg + k) + 1] * win[k];
			}
			if (cfft_forward(plan, buf, 1.0) != 0) {
				rc = -2;
				goto done;
			}
			for (k = 0; k < NFFT; k++) {
				double ar = buf[2 * k], ai = buf[2 * k + 1];
				double br = x0[2 * k], bi = x0[2 * k + 1];
				sxxc[k] += ar * ar + ai * ai;
				sx0c[2 * k] += ar * br + ai * bi;       /* Xi · conj(X0) */
				sx0c[2 * k + 1] += ai * br - ar * bi;
			}
		}
		nseg++;
	}
	if (nseg < 4) {
		rc = -1;
		goto done;
	}

	kmax = (int)floor(ATKDAQ_CAL_BAND * NFFT);
	if (kmax > NFFT / 2 - 1)
		kmax = NFFT / 2 - 1;
	{
		double p00 = 0.0;
		int m;
		for (m = -kmax; m <= kmax; m++) {
			if (abs(m) <= 2)
				continue;
			p00 += sxx[(m + NFFT) % NFFT];
			used++;
		}
		out->bins_used = used;
		out->power_db[0] = (float)(10.0 * log10(p00 / ((double)used * nseg * u) + 1e-30));
		out->spread_deg = 0.0f;

		for (ch = 1; ch < n_ch; ch++) {
			const double *sii = sxx + (size_t)ch * NFFT;
			const double *si0 = sx0 + (size_t)ch * 2 * NFFT;
			double pii = 0.0, cohsum = 0.0;
			double zr = 0.0, zi = 0.0;          /* neighbour product sum */
			double dphi, phi0, a, b, sw, swx, swy, swxx, swxy, den;
			double rr, ri, resid2, wsum, gmag, tau, phase_i;
			int prev_ok = 0, pm = 0;

			/* coherence and power */
			for (m = -kmax; m <= kmax; m++) {
				int kk = (m + NFFT) % NFFT;
				double c2;
				if (abs(m) <= 2)
					continue;
				pii += sii[kk];
				c2 = (si0[2 * kk] * si0[2 * kk] + si0[2 * kk + 1] * si0[2 * kk + 1]) /
				     (sii[kk] * sxx[kk] + 1e-300);
				cohsum += sqrt(c2);
			}
			out->coherence[ch] = (float)(cohsum / used);
			out->power_db[ch] = (float)(10.0 * log10(pii / ((double)used * nseg * u) + 1e-30));
			gmag = sqrt(pii / (p00 + 1e-300));

			/* 1. slope from neighbouring bins (never wraps) */
			for (m = -kmax; m <= kmax; m++) {
				int kk = (m + NFFT) % NFFT;
				if (abs(m) <= 2) {
					prev_ok = 0;
					continue;
				}
				if (prev_ok && m == pm + 1) {
					int kp = (pm + NFFT) % NFFT;
					/* S(m) · conj(S(m-1)) */
					double ar = si0[2 * kk], ai = si0[2 * kk + 1];
					double br = si0[2 * kp], bi = si0[2 * kp + 1];
					zr += ar * br + ai * bi;
					zi += ai * br - ar * bi;
				}
				prev_ok = 1;
				pm = m;
			}
			dphi = atan2(zi, zr);                    /* radians per bin */

			/* 2. intercept of the de-sloped spectrum, weighted */
			rr = ri = 0.0;
			for (m = -kmax; m <= kmax; m++) {
				int kk = (m + NFFT) % NFFT;
				double c, s, xr, xi;
				if (abs(m) <= 2)
					continue;
				c = cos(-dphi * m);
				s = sin(-dphi * m);
				xr = si0[2 * kk] * c - si0[2 * kk + 1] * s;
				xi = si0[2 * kk] * s + si0[2 * kk + 1] * c;
				rr += xr;
				ri += xi;
			}
			phi0 = atan2(ri, rr);

			/* 3. weighted LS refinement on the small residual phase */
			sw = swx = swy = swxx = swxy = 0.0;
			for (m = -kmax; m <= kmax; m++) {
				int kk = (m + NFFT) % NFFT;
				double c2, w, ang, xr, xi, c, s;
				if (abs(m) <= 2)
					continue;
				c2 = (si0[2 * kk] * si0[2 * kk] + si0[2 * kk + 1] * si0[2 * kk + 1]) /
				     (sii[kk] * sxx[kk] + 1e-300);
				if (c2 > 0.999999)
					c2 = 0.999999;
				w = c2 / (1.0 - c2);
				c = cos(-(phi0 + dphi * m));
				s = sin(-(phi0 + dphi * m));
				xr = si0[2 * kk] * c - si0[2 * kk + 1] * s;
				xi = si0[2 * kk] * s + si0[2 * kk + 1] * c;
				ang = atan2(xi, xr);
				sw += w;
				swx += w * m;
				swy += w * ang;
				swxx += w * (double)m * m;
				swxy += w * m * ang;
			}
			den = sw * swxx - swx * swx;
			if (den != 0.0) {
				b = (sw * swxy - swx * swy) / den;
				a = (swy - b * swx) / sw;
			} else {
				a = b = 0.0;
			}
			phase_i = phi0 + a;
			dphi += b;
			/* phase(f) = arg(g) - 2π f τ / fs, f = m·fs/NFFT  =>  slope per bin = -2π τ / NFFT */
			tau = -dphi * NFFT / (2.0 * M_PI);
			out->frac[ch] = (float)tau;
			/* w = 1/g = exp(-j·arg g) / |g| */
			out->w_re[ch] = (float)(cos(-phase_i) / gmag);
			out->w_im[ch] = (float)(sin(-phase_i) / gmag);

			/* 4. residual after the full correction */
			resid2 = wsum = 0.0;
			for (m = -kmax; m <= kmax; m++) {
				int kk = (m + NFFT) % NFFT;
				double c2, w, ang, xr, xi, c, s;
				if (abs(m) <= 2)
					continue;
				c2 = (si0[2 * kk] * si0[2 * kk] + si0[2 * kk + 1] * si0[2 * kk + 1]) /
				     (sii[kk] * sxx[kk] + 1e-300);
				if (c2 > 0.999999)
					c2 = 0.999999;
				w = c2 / (1.0 - c2);
				c = cos(-(phase_i + dphi * m));
				s = sin(-(phase_i + dphi * m));
				xr = si0[2 * kk] * c - si0[2 * kk + 1] * s;
				xi = si0[2 * kk] * s + si0[2 * kk + 1] * c;
				ang = atan2(xi, xr);
				resid2 += w * ang * ang;
				wsum += w;
			}
			out->resid_deg[ch] = (float)(wsum > 0 ? sqrt(resid2 / wsum) * 180.0 / M_PI : 180.0);
			if (out->resid_deg[ch] > out->spread_deg)
				out->spread_deg = out->resid_deg[ch];
		}
	}

	for (ch = 1; ch < n_ch; ch++)
		if (out->coherence[ch] < ATKDAQ_CAL_MIN_COH)
			rc = -3;

done:
	if (plan)
		destroy_cfft_plan(plan);
	free(win);
	free(buf);
	free(x0);
	free(sxx);
	free(sx0);
	return rc;
}
