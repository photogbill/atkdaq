/*
 * probe.c — `atkdaq probe`: one screen an operator can read, before a drive.
 *
 *   1. the serials found (and the ones missing, by name)
 *   2. a sync + calibration on the noise source: delays, confidence, residual
 *   3. throughput for --seconds: every channel's measured sample rate against
 *      the nominal, frames, drops, overruns, and whether a channel ever lost
 *      samples on its own
 *   4. --retune-test (§6 of the plan): ten retunes of ±1 MHz, a full sync
 *      after each, and whether the integer delays survived — the answer that
 *      decides whether a dwell schedule across channels is cheap
 *   5. --t1 MINUTES: T1 of the umbrella plan — repeated calibrations for that
 *      long, reporting the worst residual and how far each channel's phase
 *      wandered between calibrations (the "< 2 degrees and flat" test)
 *
 * and ends with a plain verdict: READY, or NOT READY and the reason.
 * Frames are assembled exactly as in a run, then discarded, so the numbers
 * are the numbers ATK would get.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "daq.h"
#include "devices/device.h"
#include "log.h"
#include "plat.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int atkdaq_probe_main(atkdaq_config *cfg, double seconds, int retune_test, double t1_minutes);

typedef struct probe {
	int stage;              /* 0 wait first sync, 1 throughput, 2 retune test, 3 t1, 4 done */
	int first_count;
	daq_result first;
	int have_first;
	double seconds;
	int64_t stage_t0;
	int retune_test;
	int rt_i, rt_waiting, rt_count_at;
	int32_t rt_prev[ATKDAQ_MAX_CHANNELS];
	int rt_same, rt_moved, rt_failed;
	int n_ch;
	uint64_t fc0;
	double t1_minutes;
	int t1_count_at, t1_cals;
	double t1_worst_spread, t1_worst_step_deg;
	float t1_prev_phase[ATKDAQ_MAX_CHANNELS];
	int t1_have_prev;
	char fail[320];
} probe;

static double wdeg(const daq_result *r, int i)
{
	return atan2(r->w_im[i], r->w_re[i]) * 180.0 / M_PI;
}

static int probe_step(daq *d, void *ctx)
{
	probe *p = (probe *)ctx;
	const daq_result *r = daq_last_result(d);
	int64_t now = daq_stream_ns(d);
	switch (p->stage) {
	case 0:
		if (r->count > p->first_count) {
			p->first = *r;
			p->have_first = 1;
			if (!r->ok) {
				snprintf(p->fail, sizeof(p->fail), "the first sync failed: %s", r->why);
				return 1;
			}
			p->stage = 1;
			p->stage_t0 = now;
		} else if (now > 30LL * 1000000000LL) {
			snprintf(p->fail, sizeof(p->fail), "no sync result after 30 s of streaming");
			return 1;
		}
		return 0;
	case 1:
		if ((double)(now - p->stage_t0) / 1e9 < p->seconds)
			return 0;
		if (p->retune_test) {
			int i;
			p->stage = 2;
			for (i = 0; i < p->n_ch; i++)
				p->rt_prev[i] = r->delay[i];
			p->rt_waiting = 0;
			return 0;
		}
		if (p->t1_minutes > 0) {
			p->stage = 3;
			p->stage_t0 = now;
			p->t1_count_at = r->count;
			return 0;
		}
		return 1;
	case 2:
		if (!p->rt_waiting) {
			uint64_t fc;
			if (p->rt_i >= 10) {
				if (p->t1_minutes > 0) {
					p->stage = 3;
					p->stage_t0 = now;
					p->t1_count_at = r->count;
					return 0;
				}
				return 1;
			}
			fc = p->fc0 + (p->rt_i % 2 == 0 ? 1000000ull : 0ull);
			if (daq_busy(d))
				return 0;
			p->rt_count_at = r->count;
			daq_tune(d, fc);
			p->rt_waiting = 1;
			return 0;
		}
		if (r->count > p->rt_count_at && !daq_busy(d)) {
			int i, moved = 0;
			if (!r->ok) {
				p->rt_failed++;
			} else {
				for (i = 0; i < p->n_ch; i++)
					if (r->delay[i] != p->rt_prev[i])
						moved = 1;
				if (moved)
					p->rt_moved++;
				else
					p->rt_same++;
				fprintf(stdout, "  retune %2d -> %s  delays:", p->rt_i + 1, moved ? "MOVED    " : "unchanged");
				for (i = 0; i < p->n_ch; i++) {
					fprintf(stdout, " %d", r->delay[i]);
					p->rt_prev[i] = r->delay[i];
				}
				fprintf(stdout, "\n");
				fflush(stdout);
			}
			p->rt_i++;
			p->rt_waiting = 0;
		}
		return 0;
	case 3:
		if (r->count > p->t1_count_at) {
			int i;
			p->t1_count_at = r->count;
			if (r->ok && r->kind != PROC_NONE) {
				p->t1_cals++;
				if (r->spread_deg > p->t1_worst_spread)
					p->t1_worst_spread = r->spread_deg;
				for (i = 0; i < p->n_ch; i++) {
					double ph = wdeg(r, i);
					if (p->t1_have_prev) {
						double step = fabs(fmod(ph - p->t1_prev_phase[i] + 540.0, 360.0) - 180.0);
						if (step > p->t1_worst_step_deg)
							p->t1_worst_step_deg = step;
					}
					p->t1_prev_phase[i] = (float)ph;
				}
				p->t1_have_prev = 1;
			}
		}
		return (double)(now - p->stage_t0) / 60e9 >= p->t1_minutes;
	default:
		return 1;
	}
}

int atkdaq_probe_main(atkdaq_config *cfg, double seconds, int retune_test, double t1_minutes)
{
	char err[1024], desc[1024];
	daq *d;
	probe p;
	daq_stats st;
	int rc, i, ready = 1;
	char why_not[512];

	why_not[0] = 0;
	memset(&p, 0, sizeof(p));
	p.seconds = seconds;
	p.retune_test = retune_test;
	p.n_ch = cfg->n_channels;
	p.fc0 = cfg->fc_hz;
	p.t1_minutes = t1_minutes;
	cfg->run_seconds = 0;
	cfg->max_frames = 0;
	cfg->auto_start_cal = 1;
	if (retune_test)
		cfg->retune_policy = 1;             /* a full sync after every retune: that is the test */
	if (t1_minutes > 0) {
		cfg->cal_interval_static_s = cfg->cal_interval_mobile_s = 10;
		cfg->sync_check_static_s = cfg->sync_check_mobile_s = 60;
	}

	printf("atkdaq probe\n============\n");
	if (!strcmp(cfg->device, "kraken")) {
		int n = atkdaq_kraken_enumerate();
		if (n < 0) {
			printf("VERDICT: NOT READY - this build has no KrakenSDR support\n");
			return 3;
		}
	}
	d = daq_create(cfg, "null", 0, err, sizeof(err));
	if (!d) {
		printf("VERDICT: NOT READY - %s\n", err);
		return 7;
	}
	rc = daq_start(d, err, sizeof(err));
	if (rc != 0) {
		printf("\nVERDICT: NOT READY - %s\n", err);
		daq_destroy(d);
		return rc;
	}
	daq_describe_device(d, desc, sizeof(desc));
	printf("device:     %s\n", desc);
	printf("channels:   %d, serials", cfg->n_channels);
	for (i = 0; i < cfg->n_channels; i++)
		printf(" %s", cfg->serial[i]);
	printf(" (in channel order)\n");
	fflush(stdout);
	if (retune_test)
		printf("\nretune test (10 x +/-1 MHz, full sync after each):\n");
	rc = daq_loop(d, probe_step, &p);
	daq_get_stats(d, &st);

	printf("\n");
	if (p.have_first && p.first.ok) {
		printf("sync:       delays");
		for (i = 0; i < cfg->n_channels; i++)
			printf(" %d", p.first.delay[i]);
		printf(" samples; peak-to-next %.1f dB (need >= %.0f) - %s\n", p.first.ptn_db,
		       cfg->sync_min_ptn_db, p.first.ptn_db >= cfg->sync_min_ptn_db ? "OK" : "WEAK");
		printf("frac delay:");
		for (i = 0; i < cfg->n_channels; i++)
			printf(" %+.3f", p.first.frac[i]);
		printf(" samples (measured, applied by the consumer)\n");
		printf("weights:   ");
		for (i = 0; i < cfg->n_channels; i++)
			printf(" %+.2fdB/%+.1fdeg", 20.0 * log10(hypot(p.first.w_re[i], p.first.w_im[i]) + 1e-30),
			       wdeg(&p.first, i));
		printf("\n");
		printf("noise seen: coherence");
		for (i = 1; i < cfg->n_channels; i++)
			printf(" %.3f", p.first.coherence[i]);
		printf(" (channels 1.. against 0)\n");
		printf("residual:   %.2f deg after calibration (T1 limit 2.0) - %s\n", p.first.spread_deg,
		       p.first.spread_deg < 2.0 ? "OK" : "HIGH");
		if (p.first.spread_deg >= 2.0) {
			ready = 0;
			snprintf(why_not, sizeof(why_not), "calibration residual %.2f deg", p.first.spread_deg);
		}
	} else {
		ready = 0;
		printf("sync:       FAILED - %s\n", p.fail[0] ? p.fail : (p.first.why[0] ? p.first.why : "no result"));
		snprintf(why_not, sizeof(why_not), "%s", p.fail[0] ? p.fail : "sync failed");
	}
	printf("throughput: %.1f s of stream;", st.stream_seconds);
	{
		int short_ch = -1;
		double worst = 0.0;
		for (i = 0; i < cfg->n_channels; i++) {
			double rel = st.rate_sps[i] / cfg->fs_hz - 1.0;
			printf(" ch%d %.4f MSPS", i, st.rate_sps[i] / 1e6);
			if (fabs(rel) > fabs(worst)) {
				worst = rel;
				short_ch = i;
			}
		}
		printf("\n");
		if (st.stream_seconds > 3 && short_ch >= 0 && worst < -0.01) {
			ready = 0;
			printf("            channel %d is %.1f %% short - put the Kraken alone on a root USB "
			       "port and power it from a 5 V >= 2.4 A supply\n",
			       short_ch, -worst * 100.0);
			if (!why_not[0])
				snprintf(why_not, sizeof(why_not), "channel %d short of samples", short_ch);
		}
	}
	printf("frames:     %llu assembled, %llu dropped on the pipe, %llu ring overruns\n",
	       (unsigned long long)st.frames_out, (unsigned long long)st.frames_dropped,
	       (unsigned long long)st.overruns);
	printf("alignment:  %s", st.misaligned ? "LOST - a channel lost samples on its own" : "held");
	for (i = 0; i < cfg->n_channels; i++)
		if (st.drops[i])
			printf(" [ch%d ~%u samples]", i, st.drops[i]);
	printf("\n");
	if (st.misaligned) {
		ready = 0;
		if (!why_not[0])
			snprintf(why_not, sizeof(why_not), "a channel lost samples (USB)");
	}
	printf("clipping:   worst frame per channel");
	for (i = 0; i < cfg->n_channels; i++)
		printf(" %u", st.clip_max_frame[i]);
	printf(" samples at the ADC rail%s\n",
	       st.clip_max_frame[0] > 0 ? " - lower the gain if this is the noise source" : "");
	if (retune_test) {
		printf("retunes:    delays unchanged %d/10, moved %d, failed %d\n", p.rt_same, p.rt_moved,
		       p.rt_failed);
		if (p.rt_moved == 0 && p.rt_same >= 8)
			printf("            the fast-retune hypothesis HOLDS: a retune costs a calibration (~0.1 s "
			       "blind), not a sync - a dwell schedule across channels is practical\n");
		else if (p.rt_moved > 0)
			printf("            the fast-retune hypothesis is FALSE on this unit: every retune needs a "
			       "full sync (~0.6 s blind); the dwell schedule still works, slower\n");
		else
			printf("            inconclusive - too few retunes completed\n");
	}
	if (t1_minutes > 0) {
		printf("T1:         %d calibrations over %.1f min; worst residual %.2f deg; worst phase step "
		       "between calibrations %.2f deg - %s\n",
		       p.t1_cals, t1_minutes, p.t1_worst_spread, p.t1_worst_step_deg,
		       p.t1_worst_spread < 2.0 ? "PASS (< 2 deg)" : "FAIL");
		if (p.t1_worst_spread >= 2.0) {
			ready = 0;
			if (!why_not[0])
				snprintf(why_not, sizeof(why_not), "T1 residual above 2 deg");
		}
	}
	if (ready)
		printf("\nVERDICT: READY\n");
	else
		printf("\nVERDICT: NOT READY - %s\n", why_not);
	fflush(stdout);
	daq_destroy(d);
	return rc != 0 && rc != 6 ? rc : (ready ? 0 : 1);
}
