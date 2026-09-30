/*
 * sync.c — integer sample delays between channels, from the noise source.
 *
 * With the noise source switched in, every channel sees the same wideband
 * noise, so the cross-correlation of channel i against channel 0 is a single
 * sharp spike at the lag between their sample counters. That lag is the
 * delay the assembler applies by offsetting channel i's read position (see
 * the ALIGNMENT section of atkdaq_frame.h).
 *
 * THE CONFIDENCE IS PEAK-TO-NEXT, not peak height. On white noise the spike
 * stands ~10·log10(n) dB above the correlation floor; what makes a lag
 * trustworthy is that nothing else in the search range comes close. The
 * worst channel's ratio goes into the header as sync_conf (centi-dB), and
 * sched.c additionally requires two consecutive captures to agree to the
 * sample before a new set of delays is applied.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "atkdaq_core.h"
#include "pocketfft.h"

/* "Next" is the largest correlation more than this many lags from the peak.
 * A residual FRACTIONAL delay spreads the peak into a sinc whose sidelobes
 * fall only as 1/(π·lag) — at 0.44 samples the third lag is -18 dB — so
 * excluding ±2 measured the peak's own skirt, not a competitor. At ±16 the
 * skirt is below -34 dB and what remains is noise or a genuine second peak. */
#define SYNC_EXCLUDE 16

static size_t pow2_at_least(size_t n)
{
	size_t m = 1;
	while (m < n)
		m <<= 1;
	return m;
}

int atkdaq_sync_estimate(const float *const *iq, int n_ch, int n, int max_lag,
                         atkdaq_sync_result *out)
{
	size_t m, j;
	double *x0 = NULL, *xi = NULL;
	cfft_plan plan = NULL;
	int ch, rc = 0;

	memset(out, 0, sizeof(*out));
	if (n_ch < 1 || n_ch > ATKDAQ_MAX_CHANNELS || n < 64 || max_lag < 1 || max_lag >= n)
		return -1;
	out->ptn_db[0] = 999.0f;
	out->worst_ptn_db = 999.0f;
	if (n_ch == 1)
		return 0;

	m = pow2_at_least((size_t)n + (size_t)max_lag + 1);
	x0 = (double *)calloc(2 * m, sizeof(double));
	xi = (double *)calloc(2 * m, sizeof(double));
	plan = make_cfft_plan(m);
	if (!x0 || !xi || !plan) {
		rc = -2;
		goto done;
	}
	for (j = 0; j < 2 * (size_t)n; j++)
		x0[j] = iq[0][j];
	if (cfft_forward(plan, x0, 1.0) != 0) {
		rc = -2;
		goto done;
	}

	for (ch = 1; ch < n_ch; ch++) {
		long l, best_l = 0;
		double best = -1.0, next = 0.0;
		memset(xi, 0, 2 * m * sizeof(double));
		for (j = 0; j < 2 * (size_t)n; j++)
			xi[j] = iq[ch][j];
		if (cfft_forward(plan, xi, 1.0) != 0) {
			rc = -2;
			goto done;
		}
		/* C = Xi · conj(X0) ; c[l] = sum_j xi[j+l] conj(x0[j]) */
		for (j = 0; j < m; j++) {
			double ar = xi[2 * j], ai = xi[2 * j + 1];
			double br = x0[2 * j], bi = x0[2 * j + 1];
			xi[2 * j] = ar * br + ai * bi;
			xi[2 * j + 1] = ai * br - ar * bi;
		}
		if (cfft_backward(plan, xi, 1.0 / (double)m) != 0) {
			rc = -2;
			goto done;
		}
		for (l = -max_lag; l <= max_lag; l++) {
			size_t k = (size_t)(l >= 0 ? l : (long)m + l);
			double mag = hypot(xi[2 * k], xi[2 * k + 1]);
			if (mag > best) {
				best = mag;
				best_l = l;
			}
		}
		for (l = -max_lag; l <= max_lag; l++) {
			size_t k;
			double mag;
			if (labs(l - best_l) <= SYNC_EXCLUDE)
				continue;
			k = (size_t)(l >= 0 ? l : (long)m + l);
			mag = hypot(xi[2 * k], xi[2 * k + 1]);
			if (mag > next)
				next = mag;
		}
		out->lag[ch] = (int32_t)best_l;
		out->ptn_db[ch] = (float)(next > 0 ? 20.0 * log10(best / next) : 999.0);
		if (out->ptn_db[ch] < out->worst_ptn_db)
			out->worst_ptn_db = out->ptn_db[ch];
		/* parabolic sub-sample estimate on |c|, for the record only */
		if (best_l > -max_lag && best_l < max_lag) {
			size_t km = (size_t)(best_l - 1 >= 0 ? best_l - 1 : (long)m + best_l - 1);
			size_t kp = (size_t)(best_l + 1 >= 0 ? best_l + 1 : (long)m + best_l + 1);
			double ym = hypot(xi[2 * km], xi[2 * km + 1]);
			double yp = hypot(xi[2 * kp], xi[2 * kp + 1]);
			double den = ym - 2.0 * best + yp;
			out->frac[ch] = (float)(den != 0.0 ? 0.5 * (ym - yp) / den : 0.0);
		}
	}

done:
	if (plan)
		destroy_cfft_plan(plan);
	free(x0);
	free(xi);
	return rc;
}
