/*
 * synth.c — a synthetic coherent receiver with a KNOWN ANSWER.
 *
 * The KrakenSDR is not in hand; this is how atkdaq is built and tested
 * anyway, and how every later session tests without Bill's time. It
 * implements the same device interface as kraken.c and plants everything
 * the real hardware is expected to do to the samples:
 *
 *   * per-channel STREAM START OFFSETS (synth.start) — the integer delays
 *     sync.c must find: expected delay[i] = start[0] - start[i];
 *   * per-channel RECEIVER RESPONSE H_i(f) = g_i·exp(j·φ_i)·exp(-j·2π·f·τ_i/fs)
 *     (synth.gain_db, synth.phase_deg, synth.frac) — what cal.c must find:
 *     expected w[i] = (g_0·e^{jφ_0}) / (g_i·e^{jφ_i}), frac[i] = τ_i - τ_0;
 *   * a NOISE SOURCE common to every channel, switched in place of the
 *     antennas; ON-AIR EMITTERS with a planted steering phase per element,
 *     seen through the same receivers (so after calibration their phases
 *     must come out as the planted steering phases, and nothing else);
 *   * independent receiver noise, 8-bit quantisation and clipping;
 *   * USB ARRIVAL LATENCY with jitter, per transfer, per channel;
 *   * SAMPLE LOSS on one channel at a planted time (synth.drop) — the
 *     misalignment drops.c and the policy must catch;
 *   * RETUNES that re-randomise every tuner phase (and, optionally, shift
 *     the stream offsets — synth.retune_moves_delays makes the §6
 *     fast-retune hypothesis false, so the policy's other branch is tested).
 *
 * The signal is built in the frequency domain, one block of BLOCK samples
 * at a time: a random spectrum per source, multiplied by each receiver's
 * H_i(f), inverse transformed. That makes the fractional delays and gains
 * exact per bin rather than approximated by a filter, and costs a handful
 * of FFTs per block. Blocks are independent realisations, so each source is
 * discontinuous at block boundaries — identically on every channel, which
 * is all sync and calibration care about.
 *
 * TWO PACES. Real time (the default) emits each transfer when the wall clock
 * says it would have arrived, which is what ATK's source sees from a real
 * Kraken. Fast (synth.fast = 1) emits as fast as atkdaq consumes, with the
 * arrival stamps computed from the stream position — the whole program then
 * runs faster than real time and every flag, sync and drop decision is
 * identical, because they are all made by sample index (see device.h).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../plat.h"
#include "atkdaq_core.h"
#include "device.h"
#include "pocketfft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Generator block: ~7 ms of samples, a power of two (16384 at 2.4 MSPS,
 * 2048 at 240 kHz) — the granularity at which the noise source, a retune or
 * a gain change takes effect, so it is kept short of a USB transfer. */
#define BLOCK (d->block)
#define EMITTER_BW_HZ 12500.0       /* a DMR-width signal */
#define MIN_LATENCY_NS 500000.0     /* 0.5 ms: the irreducible USB delay */

typedef struct {
	uint64_t s[4];
} rng_t;

static uint64_t splitmix(uint64_t *x)
{
	uint64_t z = (*x += 0x9E3779B97F4A7C15ULL);
	z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
	z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
	return z ^ (z >> 31);
}

static void rng_seed(rng_t *r, uint64_t seed)
{
	int i;
	for (i = 0; i < 4; i++)
		r->s[i] = splitmix(&seed);
}

static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

static uint64_t rng_u64(rng_t *r)   /* xoshiro256** */
{
	uint64_t *s = r->s;
	uint64_t result = rotl(s[1] * 5, 7) * 9;
	uint64_t t = s[1] << 17;
	s[2] ^= s[0];
	s[3] ^= s[1];
	s[1] ^= s[2];
	s[0] ^= s[3];
	s[2] ^= t;
	s[3] = rotl(s[3], 45);
	return result;
}

/* uniform in (0, 1] — never 0, so log() is safe */
static double rng_unit(rng_t *r) { return ((double)(rng_u64(r) >> 11) + 1.0) * (1.0 / 9007199254740992.0); }

/* complex normal CN(0, var): each component N(0, var/2) */
static void rng_cn(rng_t *r, double var, double *re, double *im)
{
	double rad = sqrt(-var * log(rng_unit(r)));
	double th = 2.0 * M_PI * rng_unit(r);
	*re = rad * cos(th);
	*im = rad * sin(th);
}

struct atkdaq_dev {
	atkdaq_config cfg;
	int n_ch;
	int block;
	double fs, period_ns;
	int transfer_samples;

	/* receiver response, current */
	double gain[ATKDAQ_MAX_CHANNELS], phase[ATKDAQ_MAX_CHANNELS], frac[ATKDAQ_MAX_CHANNELS];
	/* stream geometry */
	long start[ATKDAQ_MAX_CHANNELS];      /* normalised so the smallest is 0 */
	long lost[ATKDAQ_MAX_CHANNELS];       /* samples skipped so far (drops, moved delays) */
	uint64_t emitted[ATKDAQ_MAX_CHANNELS];/* raw samples handed to atkdaq */
	int drop_done[ATKDAQ_MAX_SYNTH_DROPS];

	/* generator: per-channel FIFO of quantised samples indexed by generator index g */
	uint8_t *fifo[ATKDAQ_MAX_CHANNELS];
	uint64_t fifo_cap;                    /* samples, power of two */
	uint64_t g_gen;                       /* samples generated so far (all channels) */
	cfft_plan plan;
	double *common, *emit, *chan;
	rng_t rng;

	/* requests from the assembler, applied at a block boundary */
	plat_mutex lock;
	uint64_t g_committed;                 /* index at which the NEXT request applies */
	int noise_on, req_noise;              /* req_noise: -1 none, 0/1 wanted */
	int req_retune, req_gain, req_gain_tenths;
	int gain_tenths;
	uint64_t fc_hz;

	/* run */
	plat_thread th;
	volatile int running;
	int started;
	int64_t t0_ns;
	volatile int64_t now_ns;
	atkdaq_on_data on_data;
	atkdaq_on_fault on_fault;
	void *user;
	struct atkdaq_ring *rings[ATKDAQ_MAX_CHANNELS];
	int have_rings;
	uint8_t *xfer;
};

static void apply_retune(atkdaq_dev *d)
{
	int i;
	for (i = 0; i < d->n_ch; i++)
		d->phase[i] = (rng_unit(&d->rng) * 2.0 - 1.0) * M_PI;
	if (d->cfg.synth_retune_moves_delays)
		for (i = 1; i < d->n_ch; i++)
			d->lost[i] += (long)(rng_u64(&d->rng) % 3u);   /* 0..2 samples */
}

static void apply_gain(atkdaq_dev *d, int tenths)
{
	int i;
	double step_db = (tenths - d->gain_tenths) / 10.0;
	for (i = 0; i < d->n_ch; i++) {
		/* each chain tracks the step to within a few tenths of a dB and a
		 * few degrees — which is why a gain change needs a recalibration */
		double err_db = (rng_unit(&d->rng) - 0.5) * 0.6;
		d->gain[i] *= pow(10.0, (step_db + err_db) / 20.0);
		d->phase[i] += (rng_unit(&d->rng) - 0.5) * 10.0 * M_PI / 180.0;
	}
	d->gain_tenths = tenths;
}

/* Generate one block for every channel and append it to the FIFOs. */
static void generate_block(atkdaq_dev *d)
{
	int i, k, e;
	double noise_var, floor_var;
	uint64_t g0;
	int noise_on;

	plat_mutex_lock(&d->lock);
	if (d->req_noise >= 0) {
		d->noise_on = d->req_noise;
		d->req_noise = -1;
	}
	if (d->req_retune) {
		apply_retune(d);
		d->req_retune = 0;
	}
	if (d->req_gain) {
		apply_gain(d, d->req_gain_tenths);
		d->req_gain = 0;
	}
	noise_on = d->noise_on;
	g0 = d->g_gen;
	/* anything requested from now on lands on the block after this one */
	d->g_committed = g0 + BLOCK;
	plat_mutex_unlock(&d->lock);

	/* per-bin variances for the time-domain powers asked for (see header) */
	noise_var = pow(10.0, d->cfg.synth_noise_db / 10.0) * BLOCK;
	floor_var = pow(10.0, d->cfg.synth_floor_db / 10.0) * BLOCK;

	/* the common noise source, and the on-air emitters' baseband spectra */
	for (k = 0; k < BLOCK; k++) {
		if (noise_on)
			rng_cn(&d->rng, noise_var, &d->common[2 * k], &d->common[2 * k + 1]);
		else
			d->common[2 * k] = d->common[2 * k + 1] = 0.0;
	}

	for (i = 0; i < d->n_ch; i++) {
		double *x = d->chan;
		double g = d->gain[i], ph = d->phase[i], tau = d->frac[i];
		/* sources: noise (common) + emitters (per-element steering) */
		for (k = 0; k < BLOCK; k++) {
			x[2 * k] = d->common[2 * k];
			x[2 * k + 1] = d->common[2 * k + 1];
		}
		if (!noise_on) {
			for (e = 0; e < ATKDAQ_MAX_EMITTERS; e++) {
				const synth_emitter *em = &d->cfg.synth_emitter[e];
				double t_blk, period, cr, ci;
				int kc, half, j;
				if (!em->active)
					continue;
				t_blk = (double)g0 / d->fs;
				period = em->burst_on_s + em->burst_off_s;
				if (period > 0 && fmod(t_blk, period) >= em->burst_on_s)
					continue;
				/* the emitter's spectrum is drawn ONCE per block (it is one
				 * signal) — the per-element steering is applied below */
				kc = (int)lround(em->f_offset_hz / d->fs * BLOCK);
				half = (int)ceil(EMITTER_BW_HZ / 2.0 / d->fs * BLOCK);
				cr = cos(em->steer_deg[i] * M_PI / 180.0);
				ci = sin(em->steer_deg[i] * M_PI / 180.0);
				for (j = -half; j <= half; j++) {
					int kk = ((kc + j) % BLOCK + BLOCK) % BLOCK;
					double er = d->emit[2 * (size_t)(e * BLOCK + kk)];
					double ei = d->emit[2 * (size_t)(e * BLOCK + kk) + 1];
					x[2 * kk] += er * cr - ei * ci;
					x[2 * kk + 1] += er * ci + ei * cr;
				}
			}
		}
		/* receiver response H_i(f), then this receiver's own noise */
		for (k = 0; k < BLOCK; k++) {
			int m = k < BLOCK / 2 ? k : k - BLOCK;
			double ang = ph - 2.0 * M_PI * (double)m * tau / BLOCK;
			double hr = g * cos(ang), hi = g * sin(ang);
			double xr = x[2 * k], xi = x[2 * k + 1], nr, ni;
			rng_cn(&d->rng, floor_var, &nr, &ni);
			x[2 * k] = xr * hr - xi * hi + nr;
			x[2 * k + 1] = xr * hi + xi * hr + ni;
		}
		cfft_backward(d->plan, x, 1.0 / BLOCK);
		/* quantise into the FIFO at generator positions g0 .. g0+BLOCK */
		for (k = 0; k < BLOCK; k++) {
			uint64_t pos = ((g0 + (uint64_t)k) & (d->fifo_cap - 1)) * 2;
			long qi = lround(x[2 * k] * 127.5 + 127.5);
			long qq = lround(x[2 * k + 1] * 127.5 + 127.5);
			d->fifo[i][pos] = (uint8_t)(qi < 0 ? 0 : qi > 255 ? 255 : qi);
			d->fifo[i][pos + 1] = (uint8_t)(qq < 0 ? 0 : qq > 255 ? 255 : qq);
		}
	}
	d->g_gen = g0 + BLOCK;
}

/* Draw each active emitter's spectrum for the next block (one draw shared by
 * every channel — it is one transmitter). */
static void draw_emitters(atkdaq_dev *d)
{
	int e, j;
	for (e = 0; e < ATKDAQ_MAX_EMITTERS; e++) {
		const synth_emitter *em = &d->cfg.synth_emitter[e];
		int kc, half, nb;
		double var;
		if (!em->active)
			continue;
		kc = (int)lround(em->f_offset_hz / d->fs * BLOCK);
		half = (int)ceil(EMITTER_BW_HZ / 2.0 / d->fs * BLOCK);
		nb = 2 * half + 1;
		/* time-domain power 10^(level/10) spread over nb bins */
		var = pow(10.0, em->level_db / 10.0) * (double)BLOCK * BLOCK / nb;
		for (j = -half; j <= half; j++) {
			int kk = ((kc + j) % BLOCK + BLOCK) % BLOCK;
			rng_cn(&d->rng, var, &d->emit[2 * (size_t)(e * BLOCK + kk)],
			       &d->emit[2 * (size_t)(e * BLOCK + kk) + 1]);
		}
	}
}

/* generator index of channel i's next raw sample */
static uint64_t chan_g(const atkdaq_dev *d, int i)
{
	return d->emitted[i] + (uint64_t)d->start[i] + (uint64_t)d->lost[i];
}

static void apply_drops(atkdaq_dev *d, int i)
{
	int k;
	for (k = 0; k < d->cfg.synth_n_drops; k++) {
		const synth_drop *dr = &d->cfg.synth_drop[k];
		if (d->drop_done[k] || dr->ch != i)
			continue;
		if ((double)chan_g(d, i) / d->fs >= dr->at_s) {
			d->lost[i] += dr->samples;
			d->drop_done[k] = 1;
		}
	}
}

static void gen_thread(void *arg)
{
	atkdaq_dev *d = (atkdaq_dev *)arg;
	int T = d->transfer_samples;
	while (d->running) {
		int i, best = -1;
		uint64_t best_end = 0;
		double jitter, t_arr;
		/* the channel whose next transfer ends earliest goes next, so the
		 * channels stay interleaved in stream time exactly as hardware is */
		for (i = 0; i < d->n_ch; i++) {
			uint64_t end;
			apply_drops(d, i);
			end = chan_g(d, i) + (uint64_t)T;
			if (best < 0 || end < best_end) {
				best = i;
				best_end = end;
			}
		}
		/* generate just what THIS transfer needs, before sleeping until it is
		 * due: generating for the furthest channel instead put ~40 ms of FFTs
		 * between channel 0's transfer and the other four, which a real
		 * Kraken's five simultaneous completions never show (and which made
		 * the real-time clocks disagree by that much) */
		while (d->running && d->g_gen < best_end) {
			draw_emitters(d);
			generate_block(d);
		}
		if (!d->running)
			break;
		jitter = rng_unit(&d->rng) * d->cfg.synth_jitter_us * 1000.0;
		t_arr = (double)d->t0_ns + (double)best_end * d->period_ns + MIN_LATENCY_NS + jitter;
		if (d->cfg.synth_fast) {
			/* Flow control: never overwrite what atkdaq has not consumed —
			 * and never run further ahead of it than a capture plus a few
			 * transfers. Without the second limit the generator filled the
			 * whole four-second ring, so a noise-source switch landed four
			 * seconds of samples after it was asked for, which no receiver
			 * does and which left every frame of a short fixture NOISE_ON. */
			uint64_t lead = (uint64_t)(2 * d->cfg.sync_samples + 2 * d->cfg.sync_max_lag +
			                           d->cfg.cal_samples + 3 * T) * 2;
			while (d->running && d->have_rings) {
				uint64_t fr = atkdaq_ring_free_space(d->rings[best]);
				uint64_t used = d->rings[best]->size - fr;
				if (fr >= (uint64_t)T * 2 && used < lead)
					break;
				plat_sleep_ms(1);
			}
		} else {
			plat_sleep_until_ns((int64_t)t_arr);
			t_arr = (double)plat_mono_ns();
		}
		if (!d->running)
			break;
		{
			uint64_t g = chan_g(d, best);
			int k;
			for (k = 0; k < T; k++) {
				uint64_t pos = ((g + (uint64_t)k) & (d->fifo_cap - 1)) * 2;
				d->xfer[2 * k] = d->fifo[best][pos];
				d->xfer[2 * k + 1] = d->fifo[best][pos + 1];
			}
		}
		d->now_ns = (int64_t)t_arr;
		d->on_data(d->user, best, d->xfer, (uint32_t)T * 2, (int64_t)t_arr);
		d->emitted[best] += (uint64_t)T;
	}
}

static int synth_open(atkdaq_dev **out, const atkdaq_config *cfg, char *err, int errlen)
{
	atkdaq_dev *d;
	int i;
	long mn;
	*out = NULL;
	d = (atkdaq_dev *)calloc(1, sizeof(*d));
	if (!d) {
		snprintf(err, errlen, "out of memory");
		return -1;
	}
	d->cfg = *cfg;
	d->n_ch = cfg->n_channels;
	d->fs = cfg->fs_hz;
	d->period_ns = 1e9 / d->fs;
	d->transfer_samples = cfg->transfer_bytes / 2;
	d->block = 1024;
	while (d->block < 16384 && (double)d->block < d->fs * 0.0068)
		d->block <<= 1;
	d->fc_hz = cfg->fc_hz;
	d->gain_tenths = cfg->gain_tenths;
	mn = cfg->synth_start[0];
	for (i = 0; i < d->n_ch; i++)
		if (cfg->synth_start[i] < mn)
			mn = cfg->synth_start[i];
	for (i = 0; i < d->n_ch; i++) {
		d->start[i] = cfg->synth_start[i] - mn;
		d->gain[i] = pow(10.0, cfg->synth_gain_db[i] / 20.0);
		d->phase[i] = cfg->synth_phase_deg[i] * M_PI / 180.0;
		d->frac[i] = cfg->synth_frac[i];
	}
	/* FIFO: two seconds or the largest offset plus four blocks, whichever is more */
	d->fifo_cap = 1;
	{
		uint64_t want = (uint64_t)(2.0 * d->fs);
		long maxoff = 0;
		for (i = 0; i < d->n_ch; i++)
			if (d->start[i] > maxoff)
				maxoff = d->start[i];
		if (want < (uint64_t)maxoff + 8u * BLOCK + (uint64_t)d->transfer_samples)
			want = (uint64_t)maxoff + 8u * BLOCK + (uint64_t)d->transfer_samples;
		while (d->fifo_cap < want)
			d->fifo_cap <<= 1;
	}
	for (i = 0; i < d->n_ch; i++) {
		d->fifo[i] = (uint8_t *)calloc((size_t)d->fifo_cap * 2, 1);
		if (!d->fifo[i]) {
			snprintf(err, errlen, "out of memory");
			goto fail;
		}
	}
	d->common = (double *)calloc(2 * BLOCK, sizeof(double));
	d->chan = (double *)calloc(2 * BLOCK, sizeof(double));
	d->emit = (double *)calloc((size_t)2 * BLOCK * ATKDAQ_MAX_EMITTERS, sizeof(double));
	d->xfer = (uint8_t *)malloc((size_t)d->transfer_samples * 2);
	d->plan = make_cfft_plan(BLOCK);
	if (!d->common || !d->chan || !d->emit || !d->xfer || !d->plan) {
		snprintf(err, errlen, "out of memory");
		goto fail;
	}
	rng_seed(&d->rng, (uint64_t)cfg->synth_seed * 0x2545F4914F6CDD1DULL + 1u);
	plat_mutex_init(&d->lock);
	d->req_noise = -1;
	*out = d;
	return 0;
fail:
	for (i = 0; i < d->n_ch; i++)
		free(d->fifo[i]);
	free(d->common);
	free(d->chan);
	free(d->emit);
	free(d->xfer);
	if (d->plan)
		destroy_cfft_plan(d->plan);
	free(d);
	return -1;
}

static void synth_close(atkdaq_dev *d)
{
	int i;
	if (!d)
		return;
	for (i = 0; i < d->n_ch; i++)
		free(d->fifo[i]);
	free(d->common);
	free(d->chan);
	free(d->emit);
	free(d->xfer);
	destroy_cfft_plan(d->plan);
	plat_mutex_destroy(&d->lock);
	free(d);
}

static int synth_start(atkdaq_dev *d, atkdaq_on_data on_data, atkdaq_on_fault on_fault, void *user)
{
	d->on_data = on_data;
	d->on_fault = on_fault;
	d->user = user;
	d->t0_ns = plat_mono_ns();
	d->now_ns = d->t0_ns;
	d->running = 1;
	if (plat_thread_start(&d->th, gen_thread, d) != 0) {
		d->running = 0;
		return -1;
	}
	d->started = 1;
	return 0;
}

static void synth_stop(atkdaq_dev *d)
{
	if (!d->started)
		return;
	d->running = 0;
	plat_thread_join(d->th);
	d->started = 0;
}

/* channel-0 raw index at which a request made now takes effect */
static uint64_t effective_index(atkdaq_dev *d)
{
	uint64_t g = d->g_committed;
	uint64_t off = (uint64_t)d->start[0] + (uint64_t)d->lost[0];
	return g > off ? g - off : 0;
}

static int synth_tune(atkdaq_dev *d, uint64_t fc_hz, uint64_t *n_eff)
{
	plat_mutex_lock(&d->lock);
	d->fc_hz = fc_hz;
	d->req_retune = 1;
	if (n_eff)
		*n_eff = effective_index(d);
	plat_mutex_unlock(&d->lock);
	return 0;
}

static const int R820T_GAINS[] = {0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254,
                                   280, 297, 328, 338, 364, 372, 386, 402, 421, 434, 439, 445, 480, 496};

static int nearest_gain(int tenths)
{
	int i, best = R820T_GAINS[0];
	for (i = 0; i < (int)(sizeof(R820T_GAINS) / sizeof(R820T_GAINS[0])); i++)
		if (abs(R820T_GAINS[i] - tenths) < abs(best - tenths))
			best = R820T_GAINS[i];
	return best;
}

static int synth_set_gain(atkdaq_dev *d, int tenths, int *applied, uint64_t *n_eff)
{
	int g = nearest_gain(tenths);
	plat_mutex_lock(&d->lock);
	d->req_gain = 1;
	d->req_gain_tenths = g;
	if (applied)
		*applied = g;
	if (n_eff)
		*n_eff = effective_index(d);
	plat_mutex_unlock(&d->lock);
	return 0;
}

static int synth_noise(atkdaq_dev *d, int on, uint64_t *n_eff)
{
	plat_mutex_lock(&d->lock);
	d->req_noise = on ? 1 : 0;
	if (n_eff)
		*n_eff = effective_index(d);
	plat_mutex_unlock(&d->lock);
	return 0;
}

static int synth_pll_locked(atkdaq_dev *d)
{
	(void)d;
	return 1;
}

static int64_t synth_now(atkdaq_dev *d)
{
	return d->cfg.synth_fast ? d->now_ns : plat_mono_ns();
}

static void synth_describe(atkdaq_dev *d, char *buf, int len)
{
	snprintf(buf, len, "synthetic coherent receiver, %d channels, %s, seed %u - NOT ANTENNAS",
	         d->n_ch, d->cfg.synth_fast ? "fast" : "real time", d->cfg.synth_seed);
}

static int synth_gains(atkdaq_dev *d, int *list, int max)
{
	int i, n = (int)(sizeof(R820T_GAINS) / sizeof(R820T_GAINS[0]));
	(void)d;
	for (i = 0; i < n && i < max; i++)
		list[i] = R820T_GAINS[i];
	return i;
}

static void synth_set_rings(atkdaq_dev *d, struct atkdaq_ring *const *rings, int n)
{
	int i;
	for (i = 0; i < n && i < ATKDAQ_MAX_CHANNELS; i++)
		d->rings[i] = rings[i];
	d->have_rings = 1;
}

const atkdaq_dev_ops atkdaq_synth_ops = {
	"synth", ATKDAQ_DEV_SYNTH,
	synth_open, synth_close, synth_start, synth_stop,
	synth_tune, synth_set_gain, synth_noise, synth_pll_locked,
	synth_now, synth_describe, synth_gains, synth_set_rings,
};
