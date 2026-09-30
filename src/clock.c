/*
 * clock.c — when was raw sample n digitised? (See atkdaq_core.h.)
 *
 * A transfer arrives some latency L >= 0 after its last sample was taken.
 * With P the sample period, every arrival satisfies t_j - n_j*P = a + L_j,
 * where a is the instant sample 0 was taken. The smallest residual in a
 * window is the transfer that suffered the least latency, which is the best
 * available estimate of a (+ the irreducible minimum latency, a constant of
 * order a millisecond that is the same for every channel and every frame).
 *
 * The window slides, so a slow drift between the SDR's crystal and the
 * host's clock is followed rather than accumulated; at 50 ppm over the
 * recent window used for time stamps (16 transfers, 0.9 s) that is under
 * 0.05 ms. Channels share one crystal, so the drift is common to all of them
 * and cancels in drops.c's comparison.
 */
#include <float.h>
#include <string.h>

#include "atkdaq_core.h"

void atkdaq_sclock_init(atkdaq_sclock *c, double fs)
{
	memset(c, 0, sizeof(*c));
	c->period_ns = fs > 0 ? 1e9 / fs : 0.0;
}

void atkdaq_sclock_add(atkdaq_sclock *c, uint64_t n_end, int64_t t_ns)
{
	c->n[c->head] = n_end;
	c->t[c->head] = t_ns;
	c->head = (c->head + 1) % ATKDAQ_SCLOCK_WIN;
	if (c->count < ATKDAQ_SCLOCK_WIN)
		c->count++;
}

int atkdaq_sclock_intercept(const atkdaq_sclock *c, int last, double *a_ns)
{
	int k, use, idx;
	double best = DBL_MAX;
	if (c->count == 0)
		return -1;
	use = (last <= 0 || last > c->count) ? c->count : last;
	for (k = 0; k < use; k++) {
		double r;
		idx = (c->head - 1 - k + 2 * ATKDAQ_SCLOCK_WIN) % ATKDAQ_SCLOCK_WIN;
		r = (double)c->t[idx] - (double)c->n[idx] * c->period_ns;
		if (r < best)
			best = r;
	}
	*a_ns = best;
	return 0;
}

int64_t atkdaq_sclock_time(const atkdaq_sclock *c, uint64_t n)
{
	double a;
	/* 16 transfers (~0.9 s at 256 KB): short enough that a common-mode
	 * loss moves the time stamps within a second, long enough that the
	 * minimum of the arrival jitter is found (the expected minimum of 16
	 * uniform draws is 1/17 of the spread) */
	if (atkdaq_sclock_intercept(c, 16, &a) != 0)
		return 0;
	return (int64_t)(a + (double)n * c->period_ns);
}
