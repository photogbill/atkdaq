/*
 * drops.c — has one channel lost samples the others did not? (atkdaq_core.h
 * has the model; this is the arithmetic.)
 *
 * For each channel the clock intercept over the RECENT transfers is compared
 * with the intercept recorded at the last sync. The median of those shifts
 * is the common mode; a channel whose own shift stands out from the median
 * by more than the threshold, for `persist` consecutive evaluations, has
 * lost samples on its own and the array is misaligned.
 *
 * Why the recent window and not the whole history: a loss makes every LATER
 * transfer's residual larger, but the lower envelope over a window only
 * rises once every pre-loss transfer has left it. A short window (8
 * transfers, ~0.4 s at 256 KB per transfer) sees the loss quickly; the price
 * is that its minimum is taken over fewer samples and so sits a little
 * higher, which is why the threshold is well above the arrival jitter and
 * why a shift must persist before it is believed.
 */
#include <math.h>
#include <string.h>

#include "atkdaq_core.h"

void atkdaq_drops_init(atkdaq_drops *d, int n_ch, double fs, int recent, double thresh_ns,
                       int persist)
{
	memset(d, 0, sizeof(*d));
	d->n_ch = n_ch;
	d->period_ns = fs > 0 ? 1e9 / fs : 0.0;
	d->recent = recent > 0 ? recent : 8;
	d->thresh_ns = thresh_ns > 0 ? thresh_ns : 1e6;
	d->persist = persist > 0 ? persist : 2;
}

int atkdaq_drops_rebase(atkdaq_drops *d, const atkdaq_sclock *const *clocks)
{
	int i;
	for (i = 0; i < d->n_ch; i++) {
		double a;
		/* THE SAME WINDOW AS THE UPDATE. A longer one reaches back past a
		 * loss that has just been resynchronised: its minimum is then the
		 * PRE-loss intercept, every later update sees the loss again, and
		 * the array is resynchronised for ever (found by the end-to-end
		 * test, which plants a loss and counts the syncs). The short window
		 * is noisier, but update() compares like with like and the median
		 * across channels takes out what is common. */
		if (atkdaq_sclock_intercept(clocks[i], d->recent, &a) != 0)
			return -1;
		d->ref_ns[i] = a;
		d->streak[i] = 0;
		d->est_samples[i] = 0;
	}
	d->common_samples = 0;
	d->have_ref = 1;
	return 0;
}

static double median(double *v, int n)
{
	int i, j;
	for (i = 1; i < n; i++) {           /* n <= 16: insertion sort */
		double x = v[i];
		for (j = i - 1; j >= 0 && v[j] > x; j--)
			v[j + 1] = v[j];
		v[j + 1] = x;
	}
	return (n & 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

uint32_t atkdaq_drops_update(atkdaq_drops *d, const atkdaq_sclock *const *clocks)
{
	double shift[ATKDAQ_MAX_CHANNELS], sorted[ATKDAQ_MAX_CHANNELS], med;
	uint32_t mask = 0;
	int i;
	if (!d->have_ref || d->n_ch < 2)
		return 0;
	for (i = 0; i < d->n_ch; i++) {
		double a;
		if (atkdaq_sclock_intercept(clocks[i], d->recent, &a) != 0)
			return 0;
		shift[i] = a - d->ref_ns[i];
		sorted[i] = shift[i];
	}
	med = median(sorted, d->n_ch);
	d->common_samples = (int64_t)llround(med / d->period_ns);
	for (i = 0; i < d->n_ch; i++) {
		double dev = shift[i] - med;
		if (fabs(dev) > d->thresh_ns) {
			if (d->streak[i] < 1000000)
				d->streak[i]++;
		} else {
			d->streak[i] = 0;
		}
		d->est_samples[i] = (int64_t)llround(dev / d->period_ns);
		if (d->streak[i] >= d->persist)
			mask |= 1u << i;
	}
	return mask;
}
