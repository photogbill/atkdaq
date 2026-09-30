/*
 * daq.c — the acquisition engine.
 *
 *   device reader threads ──on_data──▶ per-channel ring + arrival stamp
 *                                             │
 *   assembler thread (daq_loop) ◀─────────────┘
 *     · drains the stamps into each channel's clock (clock.c)
 *     · runs the policy (sched.c) and the noise-source procedures that
 *       measure delays and weights (sync.c, cal.c)
 *     · watches for a channel losing samples on its own (drops.c)
 *     · assembles aligned frames: channel i read from raw index
 *       s0 + delay[i], converted to int8, header + payload in ONE buffer
 *     · hands each frame to the writer — or, when the consumer has stalled
 *       and every buffer is waiting, DROPS the frame and moves on
 *   writer thread ──▶ stdout / file (one write per frame)
 *   stdin thread  ──▶ command queue (control.c)
 *
 * TWO RULES THE STRUCTURE EXISTS FOR:
 *
 *   A STALL COSTS FRAMES, NEVER ALIGNMENT. The assembler never waits on the
 *   consumer. A full output queue means the frame is counted, its sequence
 *   number is skipped (the consumer sees the gap) and s0 advances for every
 *   channel together. If the assembler itself fell four seconds behind the
 *   rings, the next frame's channels would be overwritten; that is detected
 *   and every channel skips to the same aligned position (OVERRUN).
 *
 *   EVERY DECISION IS BY SAMPLE INDEX. The noise source was on from sample
 *   n; a retune took effect at sample n; this frame spans [s0, s0 + N). Wall
 *   clocks only stamp arrival (for t_utc) and pace the policy's timers, via
 *   the channel-0 clock model. That is what lets the synthetic device run
 *   faster than real time with every flag landing on the same samples.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "atomics.h"
#include "control.h"
#include "daq.h"
#include "devices/device.h"
#include "log.h"
#include "plat.h"
#include "version.h"

#define STAMP_CAP 1024
#define MAX_POOL 256
#define CMDQ 64
#define MAX_NOISE_IV 16
#define NS 1000000000LL

typedef struct stamp {
	uint64_t n_end;
	int64_t t_ns;
} stamp;

typedef struct chan {
	atkdaq_ring ring;
	stamp stamps[STAMP_CAP];
	volatile uint64_t stamp_w;         /* producer */
	uint64_t stamp_r;                  /* consumer */
	volatile uint64_t rx_samples;      /* producer: raw samples received */
	atkdaq_sclock clock;               /* consumer */
	uint64_t last_n_end;
	int32_t delay;
	float frac, w_re, w_im;
	uint32_t drops;
	uint32_t clip_max;
} chan;

typedef struct noise_iv {
	uint64_t start, end;               /* channel-0 raw index; end = UINT64_MAX while on */
} noise_iv;

typedef struct proc {
	proc_kind kind;
	int stage;                         /* 0 idle, 1 settling, 2 capturing */
	uint64_t n_on;                     /* channel-0 index the noise source is in force from */
	uint64_t c0;                       /* channel-0 raw index of the capture start */
	int64_t cstart[ATKDAQ_MAX_CHANNELS];
	int len;                           /* samples per channel to capture */
	int64_t t_begin;
	int after_retune;
} proc;

struct daq {
	atkdaq_config cfg;
	const atkdaq_dev_ops *ops;
	atkdaq_dev *dev;
	int n;
	chan ch[ATKDAQ_MAX_CHANNELS];
	int device_open, device_started;

	/* frame geometry */
	uint32_t spc;
	size_t hdr_bytes, payload_bytes, frame_bytes;
	uint64_t settle_samples, retune_settle_samples;

	/* frame state */
	int have_start;
	uint64_t next_s0;
	uint32_t seq, segment, passport;
	uint64_t seg_start_s0;
	uint8_t cal_state;
	uint16_t spread_cdeg, sync_conf;
	int64_t cal_t_ns;                  /* stream clock of the passport's measurement */
	int cal_stale;
	uint64_t fc_hz;
	int gain_tenths;
	uint32_t pending_flags;
	int misaligned;

	/* index-keyed events */
	noise_iv noise[MAX_NOISE_IV];
	int n_noise;
	int noise_on;                      /* current device state */
	int noise_held;                    /* operator holds it (0/1), -1 = policy owns it */
	uint64_t retune_from, retune_until;
	int seg_change_pending;
	uint64_t seg_change_at, seg_new_fc;
	int seg_new_gain;
	int deferred_tune;                 /* a tune/gain waiting for a procedure to end */
	uint64_t deferred_fc;
	int deferred_gain, deferred_gain_tenths;

	/* policy and procedures */
	sched sch;
	proc pr;
	atkdaq_drops drops;
	int64_t last_drop_check_ns;
	int64_t last_common;
	daq_result last;
	uint8_t *cap_raw[ATKDAQ_MAX_CHANNELS];
	float *cap_f[ATKDAQ_MAX_CHANNELS];
	size_t cap_samples_max;

	/* output */
	uint8_t *pool[MAX_POOL];
	int pool_n;
	int q[MAX_POOL], q_head, q_count;
	int freel[MAX_POOL], free_count;
	plat_mutex q_lock;
	plat_cond q_cond;
	plat_thread writer_th;
	int writer_started;
	volatile int writer_stop, writer_failed;
	plat_out *out;
	char out_desc[520];
	uint64_t frames_out, frames_dropped, overruns;
	int64_t last_stall_log_ns;
	uint64_t stall_logged_at;

	/* commands */
	int read_stdin;
	plat_thread stdin_th;
	command cmdq[CMDQ];
	int cmd_head, cmd_count;
	plat_mutex cmd_lock;

	/* lifecycle */
	volatile int quit;
	volatile int fault;
	char fault_msg[256];
	plat_mutex wake_lock;
	plat_cond wake_cond;
	int64_t t_start_ns;
	int64_t last_status_ns;
	int exit_code;
};

/* ------------------------------------------------------------------------ */
/* producer side                                                            */
/* ------------------------------------------------------------------------ */

static void on_data(void *user, int chn, const uint8_t *buf, uint32_t len, int64_t t)
{
	daq *d = (daq *)user;
	chan *c = &d->ch[chn];
	uint64_t w, n_end;
	atkdaq_ring_write(&c->ring, buf, len);
	n_end = c->ring.wpos / 2;          /* this thread is the only writer of wpos */
	w = c->stamp_w;
	c->stamps[w & (STAMP_CAP - 1)].n_end = n_end;
	c->stamps[w & (STAMP_CAP - 1)].t_ns = t;
	atkq_store_rel(&c->stamp_w, w + 1);
	atkq_store_rel(&c->rx_samples, n_end);
	plat_cond_signal(&d->wake_cond);
}

static void on_fault(void *user, int chn, const char *what)
{
	daq *d = (daq *)user;
	(void)chn;
	snprintf(d->fault_msg, sizeof(d->fault_msg), "%s", what);
	d->fault = 1;
	plat_cond_signal(&d->wake_cond);
}

/* ------------------------------------------------------------------------ */
/* threads: writer and stdin                                                */
/* ------------------------------------------------------------------------ */

static void writer_main(void *arg)
{
	daq *d = (daq *)arg;
	for (;;) {
		int idx;
		plat_mutex_lock(&d->q_lock);
		while (d->q_count == 0 && !d->writer_stop)
			plat_cond_wait(&d->q_cond, &d->q_lock, 100);
		if (d->q_count == 0) {
			plat_mutex_unlock(&d->q_lock);
			break;
		}
		idx = d->q[d->q_head];
		d->q_head = (d->q_head + 1) % MAX_POOL;
		d->q_count--;
		plat_mutex_unlock(&d->q_lock);

		if (!d->writer_failed && plat_out_write(d->out, d->pool[idx], d->frame_bytes) != 0) {
			d->writer_failed = 1;
			d->quit = 1;
			plat_cond_signal(&d->wake_cond);
		}
		plat_mutex_lock(&d->q_lock);
		d->freel[d->free_count++] = idx;
		if (!d->writer_failed)
			d->frames_out++;
		plat_mutex_unlock(&d->q_lock);
	}
}

static void stdin_main(void *arg)
{
	daq *d = (daq *)arg;
	char line[512], err[256];
	for (;;) {
		command c;
		int n = plat_read_line(line, sizeof(line));
		int rc;
		if (n < 0) {
			/* the consumer closed our stdin: it has gone, or is telling us to */
			memset(&c, 0, sizeof(c));
			c.kind = CMD_QUIT;
		} else {
			rc = control_parse(line, &c, err, sizeof(err));
			if (rc > 0)
				continue;
			if (rc < 0) {
				log_event("error", "msg=\"%s\"", err);
				continue;
			}
		}
		plat_mutex_lock(&d->cmd_lock);
		if (d->cmd_count < CMDQ) {
			d->cmdq[(d->cmd_head + d->cmd_count) % CMDQ] = c;
			d->cmd_count++;
		}
		plat_mutex_unlock(&d->cmd_lock);
		plat_cond_signal(&d->wake_cond);
		if (c.kind == CMD_QUIT)
			break;
	}
}

/* ------------------------------------------------------------------------ */
/* clocks and time                                                          */
/* ------------------------------------------------------------------------ */

static void drain_stamps(daq *d)
{
	int i;
	for (i = 0; i < d->n; i++) {
		chan *c = &d->ch[i];
		uint64_t w = atkq_load_acq(&c->stamp_w);
		if (w - c->stamp_r > STAMP_CAP)
			c->stamp_r = w - STAMP_CAP;   /* fell a minute behind; keep the newest */
		while (c->stamp_r < w) {
			stamp s = c->stamps[c->stamp_r & (STAMP_CAP - 1)];
			atkdaq_sclock_add(&c->clock, s.n_end, s.t_ns);
			c->last_n_end = s.n_end;
			c->stamp_r++;
		}
	}
}

/* stream clock "now": the channel-0 clock at the newest sample received */
static int64_t stream_now(daq *d)
{
	if (d->ch[0].clock.count > 0)
		return atkdaq_sclock_time(&d->ch[0].clock, d->ch[0].last_n_end);
	return d->device_open ? d->ops->now_ns(d->dev) : plat_mono_ns();
}

static uint64_t rx(daq *d, int i) { return atkq_load_acq(&d->ch[i].rx_samples); }

static const atkdaq_sclock *const *clocks_of(daq *d)
{
	static const atkdaq_sclock *ptrs[ATKDAQ_MAX_CHANNELS];
	int i;
	for (i = 0; i < d->n; i++)
		ptrs[i] = &d->ch[i].clock;
	return ptrs;
}

/* coarse offset of channel i against channel 0 from the clocks: the raw
 * index on channel i digitised at the same instant as index n on channel 0
 * is n + coarse_i. `recent` transfers of each clock (0 = all). */
static int64_t clock_offset(daq *d, int i, int recent)
{
	double a0, ai;
	if (i == 0)
		return 0;
	if (atkdaq_sclock_intercept(&d->ch[0].clock, recent, &a0) != 0 ||
	    atkdaq_sclock_intercept(&d->ch[i].clock, recent, &ai) != 0)
		return 0;
	return (int64_t)llround((a0 - ai) / d->ch[0].clock.period_ns);
}

/* Where to centre channel i's sync capture. The best prior wins:
 *   aligned and synced   the current delay — exact unless something moved it
 *   misaligned           the current delay corrected by the loss drops.c
 *                        measured for that channel (a channel that lost D
 *                        samples needs its delay reduced by D) — the loss can
 *                        be far larger than the lag search
 *   never synced         the clocks' recent lower envelopes (~ms, i.e. a few
 *                        thousand samples, well inside sync_max_lag) */
static int64_t coarse_offset(daq *d, int i)
{
	if (i == 0)
		return 0;
	if (d->cal_state != ATKDAQ_CAL_NONE && d->drops.have_ref) {
		if (d->misaligned)
			return (int64_t)d->ch[i].delay - (d->drops.est_samples[i] - d->drops.est_samples[0]);
		return d->ch[i].delay;
	}
	return clock_offset(d, i, 16);
}

/* ------------------------------------------------------------------------ */
/* index-keyed flags                                                        */
/* ------------------------------------------------------------------------ */

static void noise_mark(daq *d, int on, uint64_t n_eff)
{
	if (on) {
		if (d->n_noise == MAX_NOISE_IV) {
			memmove(&d->noise[0], &d->noise[1], sizeof(noise_iv) * (MAX_NOISE_IV - 1));
			d->n_noise--;
		}
		d->noise[d->n_noise].start = n_eff;
		d->noise[d->n_noise].end = UINT64_MAX;
		d->n_noise++;
	} else if (d->n_noise > 0 && d->noise[d->n_noise - 1].end == UINT64_MAX) {
		d->noise[d->n_noise - 1].end = n_eff;
	}
}

static int noise_overlaps(daq *d, uint64_t a, uint64_t b)
{
	int k;
	for (k = 0; k < d->n_noise; k++) {
		uint64_t s = d->noise[k].start > d->settle_samples ? d->noise[k].start - d->settle_samples : 0;
		uint64_t e = d->noise[k].end == UINT64_MAX ? UINT64_MAX : d->noise[k].end + d->settle_samples;
		if (a < e && b > s)
			return 1;
	}
	return 0;
}

/* the index at which a change requested now is in force */
static uint64_t eff_index(daq *d, uint64_t reported)
{
	if (reported != ATKDAQ_UNKNOWN_INDEX)
		return reported;
	/* everything received so far was digitised before the request; what
	 * comes later may or may not have been — the settle window covers it */
	return rx(d, 0);
}

static int device_noise(daq *d, int on)
{
	uint64_t n_eff = ATKDAQ_UNKNOWN_INDEX;
	if (d->noise_on == on)
		return 0;
	if (d->ops->noise(d->dev, on, &n_eff) != 0) {
		log_event("error", "msg=\"noise source switch failed (%s)\"", on ? "on" : "off");
		return -1;
	}
	d->noise_on = on;
	noise_mark(d, on, eff_index(d, n_eff));
	log_event("noise", "state=%s at=%llu", on ? "on" : "off",
	          (unsigned long long)eff_index(d, n_eff));
	return 0;
}

/* ------------------------------------------------------------------------ */
/* noise-source procedures: sync + cal, cal only, check                     */
/* ------------------------------------------------------------------------ */

static void proc_fail(daq *d, const char *why)
{
	proc_kind k = d->pr.kind;
	if (d->noise_held < 0)
		device_noise(d, 0);
	d->pr.stage = 0;
	d->pr.kind = PROC_NONE;
	d->last.kind = k;
	d->last.ok = 0;
	d->last.delays_changed = 0;
	snprintf(d->last.why, sizeof(d->last.why), "%s", why);
	d->last.count++;
	sched_done(&d->sch, k, stream_now(d), 0, 0);
	log_event("proc_fail", "kind=%s msg=\"%s\"", sched_proc_name(k), why);
}

static void proc_begin(daq *d, proc_kind k)
{
	int i;
	int64_t now = stream_now(d);
	memset(&d->pr, 0, sizeof(d->pr));
	d->pr.kind = k;
	d->pr.t_begin = now;
	if (device_noise(d, 1) != 0) {
		d->pr.stage = 0;
		d->pr.kind = PROC_NONE;
		sched_done(&d->sch, k, now, 0, 0);
		return;
	}
	/* the interval the switch just opened (or the one already open) */
	d->pr.n_on = d->noise[d->n_noise - 1].start;
	d->pr.c0 = d->pr.n_on + d->settle_samples;
	if (d->retune_until > d->pr.c0)
		d->pr.c0 = d->retune_until;
	if (k == PROC_CAL) {
		d->pr.len = d->cfg.cal_samples;
		for (i = 0; i < d->n; i++)
			d->pr.cstart[i] = (int64_t)d->pr.c0 + d->ch[i].delay;
	} else {
		d->pr.len = 2 * d->cfg.sync_samples + 2 * d->cfg.sync_max_lag + d->cfg.cal_samples;
		for (i = 0; i < d->n; i++)
			d->pr.cstart[i] = (int64_t)d->pr.c0 + coarse_offset(d, i);
	}
	for (i = 0; i < d->n; i++)
		if (d->pr.cstart[i] < 0) {
			proc_fail(d, "capture would start before the stream did");
			return;
		}
	d->pr.stage = 1;
	log_event("proc_start", "kind=%s noise_from=%llu capture_from=%llu samples=%d",
	          sched_proc_name(k), (unsigned long long)d->pr.n_on, (unsigned long long)d->pr.c0,
	          d->pr.len);
}

static void apply_measurement(daq *d, const int32_t *delays, const atkdaq_cal_result *cal,
                              float ptn_db, int have_delays)
{
	int i, changed = 0;
	for (i = 0; i < d->n; i++) {
		if (have_delays && delays[i] != d->ch[i].delay) {
			changed = 1;
			d->ch[i].delay = delays[i];
		}
		d->ch[i].frac = cal->frac[i];
		d->ch[i].w_re = cal->w_re[i];
		d->ch[i].w_im = cal->w_im[i];
		if (have_delays)
			d->ch[i].drops = 0;
	}
	if (changed)
		d->pending_flags |= ATKDAQ_F_DISCONTINUITY;
	d->passport++;
	d->cal_state = ATKDAQ_CAL_CALIBRATED;
	d->cal_stale = 0;
	d->cal_t_ns = stream_now(d);
	{
		double cdeg = cal->spread_deg * 100.0;
		d->spread_cdeg = (uint16_t)(cdeg > 65534.0 ? 65534.0 : cdeg);
	}
	if (have_delays) {
		double c = ptn_db * 100.0;
		d->sync_conf = (uint16_t)(c > 65535.0 ? 65535.0 : c < 0 ? 0 : c);
		d->misaligned = 0;
		atkdaq_drops_rebase(&d->drops, clocks_of(d));
	}
	d->last.delays_changed = changed;
}

static void proc_finish(daq *d)
{
	proc_kind k = d->pr.kind;
	int i, ok = 1, changed = 0, moved = 0;
	int32_t delays[ATKDAQ_MAX_CHANNELS];
	atkdaq_cal_result cal;
	atkdaq_sync_result sa, sb;
	const float *ptr[ATKDAQ_MAX_CHANNELS];
	char why[256];
	int Ns = d->cfg.sync_samples, Nc = d->cfg.cal_samples, L = d->cfg.sync_max_lag;
	why[0] = 0;
	memset(&sa, 0, sizeof(sa));

	if (k == PROC_CAL) {
		for (i = 0; i < d->n; i++) {
			atkdaq_u8_to_cf32(d->cap_f[i], d->cap_raw[i], (size_t)Nc * 2);
			ptr[i] = d->cap_f[i];
			delays[i] = d->ch[i].delay;
		}
		if (atkdaq_cal_estimate(ptr, d->n, Nc, (double)d->cfg.fs_hz, &cal) != 0) {
			ok = 0;
			snprintf(why, sizeof(why), "calibration: the noise source was not seen (band coherence "
			                           "ch1..: %.2f %.2f %.2f %.2f)",
			         cal.coherence[1 % d->n], cal.coherence[2 % d->n], cal.coherence[3 % d->n],
			         cal.coherence[4 % d->n]);
		} else {
			/* After a sync every residual is within half a sample. A cal on
			 * the current delays that finds a whole sample or more has found
			 * that the integer delays MOVED: a cal-only retune that did not
			 * keep them (§6), or a loss too small for drops.c to see. The
			 * weights would still be right at one frequency, because the slope
			 * absorbs the shift, but the frames would no longer be aligned
			 * sample for sample. So this is not applied; a sync is. */
			for (i = 1; i < d->n && !moved; i++)
				if (fabs(cal.frac[i]) > ATKDAQ_FRAC_WHOLE) {
					moved = i;
					ok = 0;
					snprintf(why, sizeof(why),
					         "calibration found channel %d %.2f samples off: the integer delays have "
					         "moved - syncing instead",
					         i, cal.frac[i]);
				}
		}
	} else {
		/* two sync captures, back to back, must agree to the sample */
		for (i = 0; i < d->n; i++) {
			atkdaq_u8_to_cf32(d->cap_f[i], d->cap_raw[i], (size_t)Ns * 2);
			ptr[i] = d->cap_f[i];
		}
		if (atkdaq_sync_estimate(ptr, d->n, Ns, L, &sa) != 0) {
			proc_fail(d, "sync estimate failed (input)");
			return;
		}
		for (i = 0; i < d->n; i++) {
			atkdaq_u8_to_cf32(d->cap_f[i], d->cap_raw[i] + (size_t)Ns * 2, (size_t)Ns * 2);
			ptr[i] = d->cap_f[i];
		}
		if (atkdaq_sync_estimate(ptr, d->n, Ns, L, &sb) != 0) {
			proc_fail(d, "sync estimate failed (input)");
			return;
		}
		for (i = 0; i < d->n; i++) {
			int64_t coarse = d->pr.cstart[i] - (int64_t)d->pr.c0;
			delays[i] = (int32_t)(coarse + sa.lag[i]);
			if (d->cfg.sync_verify && sa.lag[i] != sb.lag[i]) {
				ok = 0;
				snprintf(why, sizeof(why),
				         "channel %d: the two sync captures disagree (%d vs %d samples) - refusing "
				         "to apply either",
				         i, sa.lag[i], sb.lag[i]);
			}
		}
		if (ok && sa.worst_ptn_db < d->cfg.sync_min_ptn_db) {
			ok = 0;
			snprintf(why, sizeof(why),
			         "sync peak-to-next %.1f dB is below %.1f dB - the noise source may not be "
			         "reaching every channel",
			         sa.worst_ptn_db, d->cfg.sync_min_ptn_db);
		}
		if (ok) {
			/* calibrate on the aligned tail of the same capture */
			for (i = 0; i < d->n; i++) {
				size_t off = (size_t)(2 * Ns + L + sa.lag[i]);
				atkdaq_u8_to_cf32(d->cap_f[i], d->cap_raw[i] + off * 2, (size_t)Nc * 2);
				ptr[i] = d->cap_f[i];
			}
			if (atkdaq_cal_estimate(ptr, d->n, Nc, (double)d->cfg.fs_hz, &cal) != 0) {
				ok = 0;
				snprintf(why, sizeof(why), "calibration after sync: the noise source was not seen");
			}
		}
	}
	if (ok && cal.spread_deg > d->cfg.cal_max_spread_deg) {
		ok = 0;
		snprintf(why, sizeof(why),
		         "post-calibration residual %.2f deg exceeds %.2f deg - not applied", cal.spread_deg,
		         d->cfg.cal_max_spread_deg);
	}
	if (d->noise_held < 0)
		device_noise(d, 0);
	d->pr.stage = 0;
	d->pr.kind = PROC_NONE;
	if (!ok) {
		d->last.kind = k;
		d->last.ok = 0;
		d->last.delays_changed = 0;
		snprintf(d->last.why, sizeof(d->last.why), "%s", why);
		d->last.count++;
		if (moved) {
			/* not a failure to back off from: a sync is due now */
			sched_on_delays_moved(&d->sch);
			log_event("delays_moved", "channel=%d frac=%.3f msg=\"%s\"", moved, cal.frac[moved], why);
			return;
		}
		sched_done(&d->sch, k, stream_now(d), 0, 0);
		log_event("proc_fail", "kind=%s msg=\"%s\"", sched_proc_name(k), why);
		return;
	}
	apply_measurement(d, delays, &cal, sa.worst_ptn_db, k != PROC_CAL);
	changed = d->last.delays_changed;
	d->last.kind = k;
	d->last.ok = 1;
	d->last.why[0] = 0;
	d->last.ptn_db = k != PROC_CAL ? sa.worst_ptn_db : 0.0f;
	d->last.spread_deg = cal.spread_deg;
	d->last.passport = d->passport;
	d->last.t_stream_ns = stream_now(d) - d->t_start_ns;
	for (i = 0; i < d->n; i++) {
		d->last.delay[i] = d->ch[i].delay;
		d->last.frac[i] = d->ch[i].frac;
		d->last.w_re[i] = d->ch[i].w_re;
		d->last.w_im[i] = d->ch[i].w_im;
		d->last.coherence[i] = cal.coherence[i];
	}
	d->last.count++;
	sched_done(&d->sch, k, stream_now(d), 1, changed);
	{
		char dl[256], fr[256], wa[256], wp[256];
		int o1 = 0, o2 = 0, o3 = 0, o4 = 0;
		for (i = 0; i < d->n; i++) {
			double re = d->ch[i].w_re, im = d->ch[i].w_im;
			o1 += snprintf(dl + o1, sizeof(dl) - o1, "%s%d", i ? "," : "", d->ch[i].delay);
			o2 += snprintf(fr + o2, sizeof(fr) - o2, "%s%.4f", i ? "," : "", d->ch[i].frac);
			o3 += snprintf(wa + o3, sizeof(wa) - o3, "%s%.3f", i ? "," : "",
			               20.0 * log10(hypot(re, im) + 1e-30));
			o4 += snprintf(wp + o4, sizeof(wp) - o4, "%s%.2f", i ? "," : "",
			               atan2(im, re) * 180.0 / 3.14159265358979323846);
		}
		log_event(k == PROC_CAL ? "cal" : k == PROC_CHECK ? "check" : "sync",
		          "passport=%u delays=%s changed=%d frac=%s w_db=%s w_deg=%s spread_deg=%.3f "
		          "ptn_db=%.1f",
		          d->passport, dl, changed, fr, wa, wp, cal.spread_deg,
		          k != PROC_CAL ? sa.worst_ptn_db : 0.0);
	}
}

static void proc_step(daq *d)
{
	int i;
	if (d->pr.stage == 0)
		return;
	/* give up if the device stopped delivering */
	if (stream_now(d) - d->pr.t_begin > 10 * NS) {
		proc_fail(d, "timed out waiting for the capture");
		return;
	}
	if (d->pr.stage == 1) {
		for (i = 0; i < d->n; i++)
			if (rx(d, i) < (uint64_t)d->pr.cstart[i] + (uint64_t)d->pr.len)
				return;
		/* every channel holds the whole window: copy it out */
		for (i = 0; i < d->n; i++) {
			int rc = atkdaq_ring_read(&d->ch[i].ring, (uint64_t)d->pr.cstart[i] * 2, d->cap_raw[i],
			                          (size_t)d->pr.len * 2);
			if (rc != 0) {
				proc_fail(d, rc == -2 ? "the capture was overwritten before it could be read"
				                      : "capture read failed");
				return;
			}
		}
		proc_finish(d);
	}
}

/* ------------------------------------------------------------------------ */
/* retune and gain                                                          */
/* ------------------------------------------------------------------------ */

static void do_tune(daq *d, uint64_t fc)
{
	uint64_t n_eff = ATKDAQ_UNKNOWN_INDEX;
	if (d->ops->tune(d->dev, fc, &n_eff) != 0) {
		log_event("error", "msg=\"retune to %llu Hz failed on at least one channel\"",
		          (unsigned long long)fc);
		return;
	}
	n_eff = eff_index(d, n_eff);
	d->seg_change_pending = 1;
	d->seg_change_at = n_eff;
	d->seg_new_fc = fc;
	d->seg_new_gain = d->gain_tenths;
	d->retune_from = n_eff;
	d->retune_until = n_eff + d->retune_settle_samples;
	if (d->cal_state == ATKDAQ_CAL_CALIBRATED)
		d->cal_state = ATKDAQ_CAL_SYNCED;       /* delays hold (maybe); weights do not */
	d->cal_stale = 1;
	sched_on_retune(&d->sch, stream_now(d));
	log_event("retune", "fc_hz=%llu at=%llu", (unsigned long long)fc, (unsigned long long)n_eff);
}

static void do_gain(daq *d, int tenths)
{
	uint64_t n_eff = ATKDAQ_UNKNOWN_INDEX;
	int applied = tenths;
	if (d->ops->set_gain(d->dev, tenths, &applied, &n_eff) != 0) {
		log_event("error", "msg=\"gain %d failed on at least one channel\"", tenths);
		return;
	}
	n_eff = eff_index(d, n_eff);
	/* a retune still waiting for its first frame keeps its new frequency */
	if (!d->seg_change_pending)
		d->seg_new_fc = d->fc_hz;
	d->seg_change_pending = 1;
	d->seg_change_at = n_eff;
	d->seg_new_gain = applied;
	d->retune_from = n_eff;
	d->retune_until = n_eff + d->retune_settle_samples;
	if (d->cal_state == ATKDAQ_CAL_CALIBRATED)
		d->cal_state = ATKDAQ_CAL_SYNCED;
	d->cal_stale = 1;
	sched_on_gain(&d->sch, stream_now(d));
	log_event("gain", "tenths_db=%d applied=%d at=%llu", tenths, applied, (unsigned long long)n_eff);
}

int daq_tune(daq *d, uint64_t fc)
{
	if (d->pr.stage != 0) {
		d->deferred_tune = 1;
		d->deferred_fc = fc;
		return 0;
	}
	do_tune(d, fc);
	return 0;
}

/* ------------------------------------------------------------------------ */
/* commands                                                                 */
/* ------------------------------------------------------------------------ */

void daq_hold_noise(daq *d, int state)
{
	if (state == 2) {
		d->noise_held = -1;
		sched_hold_noise(&d->sch, 0);
		if (d->pr.stage == 0)
			device_noise(d, 0);
		return;
	}
	if (d->pr.stage != 0)
		proc_fail(d, "the operator took the noise source");
	d->noise_held = state ? 1 : 0;
	sched_hold_noise(&d->sch, 1);
	device_noise(d, state ? 1 : 0);
}

static void status_line(daq *d);

static void handle_command(daq *d, const command *c)
{
	switch (c->kind) {
	case CMD_TUNE:
		daq_tune(d, (uint64_t)c->ival);
		break;
	case CMD_GAIN:
		if (d->pr.stage != 0) {
			d->deferred_gain = 1;
			d->deferred_gain_tenths = (int)c->ival;
		} else {
			do_gain(d, (int)c->ival);
		}
		break;
	case CMD_SYNC:
		sched_request(&d->sch, PROC_FULL);
		break;
	case CMD_CAL:
		sched_request(&d->sch, PROC_CAL);
		break;
	case CMD_CHECK:
		sched_request(&d->sch, PROC_CHECK);
		break;
	case CMD_NOISE:
		daq_hold_noise(d, (int)c->ival);
		break;
	case CMD_MODE:
		sched_set_mode(&d->sch, (int)c->ival);
		log_event("mode", "mode=%s", c->ival ? "mobile" : "static");
		break;
	case CMD_QUIET:
		sched_on_quiet(&d->sch);
		break;
	case CMD_INTERVAL:
		sched_set_interval(&d->sch, c->what, (int)c->ival);
		log_event("interval", "what=%s seconds=%d", c->what ? "cal" : "sync_check", (int)c->ival);
		break;
	case CMD_STATUS:
		status_line(d);
		break;
	case CMD_QUIT:
		d->quit = 1;
		break;
	default:
		break;
	}
}

static void process_commands(daq *d)
{
	for (;;) {
		command c;
		plat_mutex_lock(&d->cmd_lock);
		if (d->cmd_count == 0) {
			plat_mutex_unlock(&d->cmd_lock);
			return;
		}
		c = d->cmdq[d->cmd_head];
		d->cmd_head = (d->cmd_head + 1) % CMDQ;
		d->cmd_count--;
		plat_mutex_unlock(&d->cmd_lock);
		handle_command(d, &c);
	}
}

void daq_request(daq *d, proc_kind k) { sched_request(&d->sch, k); }
int daq_busy(daq *d) { return d->pr.stage != 0 || d->seg_change_pending || d->deferred_tune; }
const daq_result *daq_last_result(daq *d) { return &d->last; }
int64_t daq_stream_ns(daq *d) { return stream_now(d) - d->t_start_ns; }

void daq_describe_device(daq *d, char *buf, int len)
{
	if (d->device_open)
		d->ops->describe(d->dev, buf, len);
	else
		snprintf(buf, len, "(not open)");
}

void daq_get_stats(daq *d, daq_stats *s)
{
	int i;
	double secs = (double)(stream_now(d) - d->t_start_ns) / 1e9;
	memset(s, 0, sizeof(*s));
	plat_mutex_lock(&d->q_lock);
	s->frames_out = d->frames_out;
	plat_mutex_unlock(&d->q_lock);
	s->frames_dropped = d->frames_dropped;
	s->overruns = d->overruns;
	s->misaligned = d->misaligned;
	s->stream_seconds = secs;
	for (i = 0; i < d->n; i++) {
		s->rx_samples[i] = rx(d, i);
		s->rate_sps[i] = secs > 0 ? (double)s->rx_samples[i] / secs : 0.0;
		s->drops[i] = d->ch[i].drops;
		s->clip_max_frame[i] = d->ch[i].clip_max;
	}
}

/* ------------------------------------------------------------------------ */
/* frames                                                                   */
/* ------------------------------------------------------------------------ */

static const char *cal_name(int s)
{
	switch (s) {
	case ATKDAQ_CAL_NONE: return "none";
	case ATKDAQ_CAL_SYNCED: return "synced";
	case ATKDAQ_CAL_CALIBRATED: return "calibrated";
	case ATKDAQ_CAL_MISALIGNED: return "misaligned";
	case ATKDAQ_CAL_CALIBRATING: return "calibrating";
	default: return "?";
	}
}

static void establish_start(daq *d)
{
	int i;
	int64_t need = 0;
	for (i = 0; i < d->n; i++)
		if (d->ch[i].clock.count < 2)
			return;
	/* coarse alignment from the clocks, so frames are within a few hundred
	 * samples even before the first sync (still flagged MISALIGNED) and the
	 * sync search is centred */
	for (i = 0; i < d->n; i++) {
		d->ch[i].delay = (int32_t)clock_offset(d, i, 0);
		if (-(int64_t)d->ch[i].delay > need)
			need = -(int64_t)d->ch[i].delay;
	}
	d->next_s0 = (uint64_t)need;
	d->seg_start_s0 = d->next_s0;
	d->have_start = 1;
	d->pending_flags |= ATKDAQ_F_SEGMENT_START;
	log_event("streaming", "first_sample=%llu coarse_delays_from_clocks=1",
	          (unsigned long long)d->next_s0);
}

static int frame_ready(daq *d)
{
	int i;
	for (i = 0; i < d->n; i++) {
		int64_t end = (int64_t)d->next_s0 + d->ch[i].delay + (int64_t)d->spc;
		if (end < 0 || (uint64_t)end > rx(d, i))
			return 0;
	}
	return 1;
}

static void handle_overrun(daq *d)
{
	/* skip every channel to the same aligned position, half a second inside
	 * what is still in the rings */
	uint64_t s0 = d->next_s0;
	int i;
	for (i = 0; i < d->n; i++) {
		uint64_t oldest = atkdaq_ring_oldest(&d->ch[i].ring) / 2 + d->cfg.fs_hz / 2;
		int64_t cand = (int64_t)oldest - d->ch[i].delay;
		if (cand > (int64_t)s0)
			s0 = (uint64_t)cand;
	}
	d->overruns++;
	log_event("overrun", "skipped_samples=%llu msg=\"the assembler fell behind the rings; every "
	                     "channel skipped to the same aligned position\"",
	          (unsigned long long)(s0 - d->next_s0));
	d->next_s0 = s0;
	d->pending_flags |= ATKDAQ_F_OVERRUN | ATKDAQ_F_DISCONTINUITY;
}

static void assemble_frame(daq *d)
{
	uint64_t s0 = d->next_s0, s1 = s0 + d->spc;
	int idx = -1, i;
	uint8_t *buf, *payload;
	atkdaq_hdr h;
	atkdaq_ch chs[ATKDAQ_MAX_CHANNELS];
	uint32_t flags;
	int64_t t_mono;

	/* segment boundary: the first frame holding any post-change sample */
	if (d->seg_change_pending && s1 > d->seg_change_at) {
		d->segment++;
		d->seg_start_s0 = s0;
		d->fc_hz = d->seg_new_fc;
		d->gain_tenths = d->seg_new_gain;
		d->seg_change_pending = 0;
		d->pending_flags |= ATKDAQ_F_SEGMENT_START;
	}

	plat_mutex_lock(&d->q_lock);
	if (d->free_count > 0)
		idx = d->freel[--d->free_count];
	plat_mutex_unlock(&d->q_lock);

	if (idx < 0) {
		/* the consumer has stalled: this frame is dropped, never delayed */
		d->frames_dropped++;
		d->seq++;
		d->next_s0 = s1;
		d->pending_flags |= ATKDAQ_F_DISCONTINUITY;
		if (d->frames_dropped - d->stall_logged_at >= 20 || d->stall_logged_at == 0) {
			log_event("stall", "dropped_total=%llu msg=\"consumer is not reading; frames dropped, "
			                   "alignment kept\"",
			          (unsigned long long)d->frames_dropped);
			d->stall_logged_at = d->frames_dropped;
		}
		return;
	}
	buf = d->pool[idx];
	payload = buf + d->hdr_bytes;
	memset(chs, 0, sizeof(chs));
	for (i = 0; i < d->n; i++) {
		uint32_t clip = 0;
		uint64_t pos = (uint64_t)((int64_t)s0 + d->ch[i].delay) * 2;
		int rc = atkdaq_ring_read_ci8(&d->ch[i].ring, pos, (int8_t *)(payload + (size_t)i * d->spc * 2),
		                              (size_t)d->spc * 2, &clip);
		if (rc != 0) {
			plat_mutex_lock(&d->q_lock);
			d->freel[d->free_count++] = idx;
			plat_mutex_unlock(&d->q_lock);
			if (rc == -2)
				handle_overrun(d);
			return;
		}
		chs[i].clip = clip;
		if (clip > d->ch[i].clip_max)
			d->ch[i].clip_max = clip;
	}

	flags = d->pending_flags;
	d->pending_flags = 0;
	if (d->ops->kind == ATKDAQ_DEV_SYNTH)
		flags |= ATKDAQ_F_SYNTHETIC;
	if (d->misaligned || d->cal_state == ATKDAQ_CAL_NONE)
		flags |= ATKDAQ_F_MISALIGNED;
	if (noise_overlaps(d, s0, s1))
		flags |= ATKDAQ_F_NOISE_ON;
	if (s0 < d->retune_until && s1 > d->retune_from)
		flags |= ATKDAQ_F_RETUNE;
	if (d->cal_stale)
		flags |= ATKDAQ_F_CAL_STALE;
	for (i = 0; i < d->n; i++)
		if ((int)chs[i].clip >= d->cfg.clip_flag_thresh)
			flags |= ATKDAQ_F_CLIPPED;

	memset(&h, 0, sizeof(h));
	h.fmt = ATKDAQ_FMT_CI8;
	if (d->pr.stage != 0 && noise_overlaps(d, s0, s1))
		h.cal_state = ATKDAQ_CAL_CALIBRATING;
	else if (d->misaligned)
		h.cal_state = ATKDAQ_CAL_MISALIGNED;
	else
		h.cal_state = d->cal_state;
	h.device_kind = (uint8_t)d->ops->kind;
	h.flags = flags;
	h.seq = d->seq;
	t_mono = atkdaq_sclock_time(&d->ch[0].clock, s0);
	h.t_utc_ns = t_mono + (plat_utc_ns() - plat_mono_ns());
	h.fc_hz = d->fc_hz;
	h.fs_hz = d->cfg.fs_hz;
	h.samples_per_ch = d->spc;
	h.segment = d->segment;
	h.passport = d->passport;
	if (d->cal_state == ATKDAQ_CAL_NONE) {
		h.cal_age_ms = 0;
		h.spread_cdeg = 0xFFFF;
	} else {
		int64_t age = t_mono - d->cal_t_ns;
		h.cal_age_ms = age > 0 ? (uint32_t)(age / 1000000) : 0;
		h.spread_cdeg = d->spread_cdeg;
	}
	h.sync_conf = d->sync_conf;
	h.gain_tenths_db = (int16_t)d->gain_tenths;
	h.mode = (uint16_t)d->sch.mode;
	h.stream_sample0 = s0;
	h.seg_sample0 = s0 - d->seg_start_s0;
	for (i = 0; i < d->n; i++) {
		chs[i].delay = d->ch[i].delay;
		chs[i].frac_delay = d->ch[i].frac;
		chs[i].w_re = d->ch[i].w_re;
		chs[i].w_im = d->ch[i].w_im;
		chs[i].drops = d->ch[i].drops;
		chs[i].sample_count = rx(d, i);
	}
	atkdaq_hdr_pack(buf, d->hdr_bytes, &h, chs, d->n);

	plat_mutex_lock(&d->q_lock);
	d->q[(d->q_head + d->q_count) % MAX_POOL] = idx;
	d->q_count++;
	plat_cond_signal(&d->q_cond);
	plat_mutex_unlock(&d->q_lock);

	d->seq++;
	d->next_s0 = s1;
}

static void update_floors(daq *d)
{
	int i;
	for (i = 0; i < d->n; i++) {
		int64_t f = (int64_t)d->next_s0 + d->ch[i].delay;
		if (d->pr.stage != 0 && d->pr.cstart[i] < f)
			f = d->pr.cstart[i];
		if (f < 0)
			f = 0;
		atkdaq_ring_set_floor(&d->ch[i].ring, (uint64_t)f * 2);
	}
}

/* Where did channel i's loss happen? Walk its clock history from the newest
 * transfer backwards while each residual still stands out from its reference
 * by more than half the threshold (net of the common mode): the oldest such
 * transfer is the first one after the loss, so every sample before the end
 * of the one before it was good. Returned in channel-0 samples. */
static uint64_t loss_since(daq *d, int i)
{
	const atkdaq_sclock *c = &d->ch[i].clock;
	double P = c->period_ns, common = (double)d->drops.common_samples * P;
	int k, idx, first_bad = -1;
	uint64_t n_good = 0;
	for (k = 0; k < c->count; k++) {
		double r;
		idx = (c->head - 1 - k + 2 * ATKDAQ_SCLOCK_WIN) % ATKDAQ_SCLOCK_WIN;
		r = (double)c->t[idx] - (double)c->n[idx] * P - d->drops.ref_ns[i] - common;
		if (fabs(r) > d->drops.thresh_ns * 0.5)
			first_bad = idx;
		else
			break;
	}
	if (first_bad < 0)
		return rx(d, 0);
	idx = (first_bad - 1 + ATKDAQ_SCLOCK_WIN) % ATKDAQ_SCLOCK_WIN;
	n_good = c->n[idx];
	/* channel i index -> channel 0 index under the delay in force before the loss */
	return (int64_t)n_good - d->ch[i].delay > 0 ? (uint64_t)((int64_t)n_good - d->ch[i].delay) : 0;
}

static void check_drops(daq *d)
{
	int64_t now = stream_now(d);
	uint32_t mask;
	int i;
	if (now - d->last_drop_check_ns < NS / 10)
		return;
	d->last_drop_check_ns = now;
	if (!d->drops.have_ref || d->pr.stage != 0)
		return;
	mask = atkdaq_drops_update(&d->drops, clocks_of(d));
	if (llabs(d->drops.common_samples - d->last_common) >
	    (long long)(d->cfg.drop_thresh_us * (double)d->cfg.fs_hz / 1e6)) {
		log_event("common_loss", "samples=%lld msg=\"every channel lost the same samples (a host "
		                         "or hub stall); alignment kept, time continuity broken\"",
		          (long long)(d->drops.common_samples - d->last_common));
		d->pending_flags |= ATKDAQ_F_OVERRUN | ATKDAQ_F_DISCONTINUITY;
		d->last_common = d->drops.common_samples;
	}
	if (mask && !d->misaligned) {
		char lst[256];
		int o = 0;
		uint64_t since = UINT64_MAX;
		lst[0] = 0;
		for (i = 0; i < d->n; i++)
			if (mask & (1u << i)) {
				int64_t s = d->drops.est_samples[i];
				uint64_t si = loss_since(d, i);
				d->ch[i].drops += (uint32_t)(s > 0 ? s : -s);
				o += snprintf(lst + o, sizeof(lst) - o, "%s%d:%lld", o ? "," : "", i, (long long)s);
				if (si < since)
					since = si;
			}
		d->misaligned = 1;
		sched_on_misaligned(&d->sch, now);
		/* `since` is where the loss happened (to one USB transfer), in
		 * channel-0 samples; frames already sent that reach past it were
		 * sent as aligned and are NOT — a consumer marks what it computed
		 * from them (same passport, stream_sample0 + N > since) as suspect.
		 * `from_s0` is the first frame this program will flag itself. */
		log_event("misaligned", "channels_samples=%s since=%llu from_s0=%llu msg=\"a channel lost "
		                        "samples on its own; the array is misaligned from 'since' until the "
		                        "next sync\"",
		          lst, (unsigned long long)since, (unsigned long long)d->next_s0);
	}
}

static void status_line(daq *d)
{
	daq_stats s;
	char rates[256];
	int i, o = 0;
	daq_get_stats(d, &s);
	for (i = 0; i < d->n; i++)
		o += snprintf(rates + o, sizeof(rates) - o, "%s%.0f", i ? "," : "", s.rate_sps[i]);
	log_event("status",
	          "t=%.1f frames=%llu dropped=%llu overruns=%llu seq=%u cal_state=%s passport=%u "
	          "spread_deg=%.2f misaligned=%d noise=%d fc_hz=%llu gain_tenths=%d mode=%s segment=%u "
	          "rates=%s queue=%d",
	          s.stream_seconds, (unsigned long long)s.frames_out, (unsigned long long)s.frames_dropped,
	          (unsigned long long)s.overruns, d->seq, cal_name(d->cal_state), d->passport,
	          d->cal_state == ATKDAQ_CAL_NONE ? -1.0 : d->spread_cdeg / 100.0, d->misaligned,
	          d->noise_on, (unsigned long long)d->fc_hz, d->gain_tenths,
	          d->sch.mode ? "mobile" : "static", d->segment, rates, d->q_count);
}

/* ------------------------------------------------------------------------ */
/* lifecycle                                                                */
/* ------------------------------------------------------------------------ */

daq *daq_create(const atkdaq_config *cfg, const char *out, int read_stdin, char *err, int errlen)
{
	daq *d = (daq *)calloc(1, sizeof(*d));
	int i;
	uint64_t ring_bytes = 1;
	size_t caps;
	if (!d) {
		snprintf(err, errlen, "out of memory");
		return NULL;
	}
	d->cfg = *cfg;
	d->n = cfg->n_channels;
	d->ops = !strcmp(cfg->device, "synth") ? &atkdaq_synth_ops : &atkdaq_kraken_ops;
	d->spc = (uint32_t)((uint64_t)cfg->fs_hz * (uint64_t)cfg->frame_ms / 1000u);
	d->hdr_bytes = ATKDAQ_HDR_BYTES(d->n);
	d->payload_bytes = ATKDAQ_PAYLOAD_BYTES(d->n, d->spc);
	d->frame_bytes = d->hdr_bytes + d->payload_bytes;
	d->settle_samples = (uint64_t)cfg->settle_ms * cfg->fs_hz / 1000u;
	d->retune_settle_samples = (uint64_t)cfg->retune_settle_ms * cfg->fs_hz / 1000u;
	d->fc_hz = cfg->fc_hz;
	d->gain_tenths = cfg->gain_tenths;
	d->noise_held = -1;
	d->cal_state = ATKDAQ_CAL_NONE;
	d->spread_cdeg = 0xFFFF;
	d->read_stdin = read_stdin;
	sched_init(&d->sch, cfg);
	atkdaq_drops_init(&d->drops, d->n, cfg->fs_hz, 8, cfg->drop_thresh_us * 1000.0, cfg->drop_persist);

	while (ring_bytes < (uint64_t)(cfg->ring_seconds * cfg->fs_hz * 2.0))
		ring_bytes <<= 1;
	for (i = 0; i < d->n; i++) {
		if (atkdaq_ring_init(&d->ch[i].ring, ring_bytes) != 0) {
			snprintf(err, errlen, "cannot allocate %llu-byte ring for channel %d",
			         (unsigned long long)ring_bytes, i);
			goto fail;
		}
		atkdaq_sclock_init(&d->ch[i].clock, cfg->fs_hz);
		d->ch[i].w_re = 1.0f;
	}
	caps = (size_t)(2 * cfg->sync_samples + 2 * cfg->sync_max_lag + cfg->cal_samples);
	if ((size_t)cfg->cal_samples > caps)
		caps = (size_t)cfg->cal_samples;
	d->cap_samples_max = caps;
	for (i = 0; i < d->n; i++) {
		d->cap_raw[i] = (uint8_t *)malloc(caps * 2);
		d->cap_f[i] = (float *)malloc(caps * 2 * sizeof(float));
		if (!d->cap_raw[i] || !d->cap_f[i]) {
			snprintf(err, errlen, "out of memory (capture buffers)");
			goto fail;
		}
	}
	d->pool_n = cfg->out_queue_frames;
	for (i = 0; i < d->pool_n; i++) {
		d->pool[i] = (uint8_t *)malloc(d->frame_bytes);
		if (!d->pool[i]) {
			snprintf(err, errlen, "out of memory (output frames)");
			goto fail;
		}
		d->freel[d->free_count++] = i;
	}
	plat_mutex_init(&d->q_lock);
	plat_cond_init(&d->q_cond);
	plat_mutex_init(&d->cmd_lock);
	plat_mutex_init(&d->wake_lock);
	plat_cond_init(&d->wake_cond);

	if (!out || !strcmp(out, "-") || !out[0]) {
		plat_binary_stdio();
		d->out = plat_out_stdout();
		snprintf(d->out_desc, sizeof(d->out_desc), "stdout");
	} else if (!strcmp(out, "null")) {
		d->out = plat_out_null();
		snprintf(d->out_desc, sizeof(d->out_desc), "discarded");
	} else {
		d->out = plat_out_open_file(out);
		if (!d->out) {
			snprintf(err, errlen, "cannot create %s", out);
			goto fail;
		}
		snprintf(d->out_desc, sizeof(d->out_desc), "%s", out);
	}
	return d;
fail:
	daq_destroy(d);
	return NULL;
}

int daq_start(daq *d, char *err, int errlen)
{
	char desc[1024];
	if (d->ops->open(&d->dev, &d->cfg, err, errlen) != 0)
		return 3;
	d->device_open = 1;
	{
		struct atkdaq_ring *rings[ATKDAQ_MAX_CHANNELS];
		int i;
		for (i = 0; i < d->n; i++)
			rings[i] = &d->ch[i].ring;
		d->ops->set_rings(d->dev, rings, d->n);
	}
	d->ops->describe(d->dev, desc, sizeof(desc));
	log_event("device", "kind=%s channels=%d fc_hz=%llu fs_hz=%u gain_tenths=%d frame_ms=%d "
	                    "samples_per_ch=%u out=\"%s\" desc=\"%s\"",
	          d->ops->name, d->n, (unsigned long long)d->fc_hz, d->cfg.fs_hz, d->gain_tenths,
	          d->cfg.frame_ms, d->spc, d->out_desc, desc);
	if (plat_thread_start(&d->writer_th, writer_main, d) != 0) {
		snprintf(err, errlen, "cannot start the writer thread");
		return 7;
	}
	d->writer_started = 1;
	if (d->read_stdin && plat_thread_start(&d->stdin_th, stdin_main, d) != 0) {
		snprintf(err, errlen, "cannot start the command thread");
		return 7;
	}
	if (d->ops->start(d->dev, on_data, on_fault, d) != 0) {
		snprintf(err, errlen, "the device would not start streaming");
		return 4;
	}
	d->device_started = 1;
	d->t_start_ns = d->ops->now_ns(d->dev);
	d->last_status_ns = d->t_start_ns;
	return 0;
}

int daq_loop(daq *d, int (*stop_fn)(daq *d, void *ctx), void *ctx)
{
	int started_policy = 0;
	while (!d->quit) {
		int64_t now;
		drain_stamps(d);
		process_commands(d);
		if (d->fault) {
			log_event("fault", "msg=\"%s\"", d->fault_msg);
			d->exit_code = 5;
			break;
		}
		if (d->writer_failed) {
			/* the consumer closed the pipe: it has gone, so do we, quietly */
			d->exit_code = 6;
			break;
		}
		if (!d->have_start) {
			establish_start(d);
		} else {
			now = stream_now(d);
			if (!started_policy) {
				sched_on_start(&d->sch, now, d->cfg.auto_start_cal);
				started_policy = 1;
			}
			check_drops(d);
			if (d->pr.stage == 0) {
				if (d->deferred_tune) {
					d->deferred_tune = 0;
					do_tune(d, d->deferred_fc);
				}
				if (d->deferred_gain) {
					d->deferred_gain = 0;
					do_gain(d, d->deferred_gain_tenths);
				}
				if (d->pr.stage == 0) {
					proc_kind k = sched_next(&d->sch, now);
					if (k != PROC_NONE)
						proc_begin(d, k);
				}
			}
			proc_step(d);
			while (!d->quit && frame_ready(d)) {
				assemble_frame(d);
				if (d->cfg.max_frames > 0 && (long long)d->seq >= d->cfg.max_frames) {
					d->quit = 1;
					break;
				}
			}
			update_floors(d);
			if (now - d->last_status_ns >= NS) {
				d->last_status_ns = now;
				status_line(d);
			}
			if (d->cfg.run_seconds > 0 && (double)(now - d->t_start_ns) / 1e9 >= d->cfg.run_seconds)
				d->quit = 1;
			if (stop_fn && stop_fn(d, ctx))
				d->quit = 1;
		}
		if (!d->quit && !frame_ready(d)) {
			plat_mutex_lock(&d->wake_lock);
			plat_cond_wait(&d->wake_cond, &d->wake_lock, 5);
			plat_mutex_unlock(&d->wake_lock);
		}
	}
	return d->exit_code;
}

void daq_destroy(daq *d)
{
	int i;
	if (!d)
		return;
	if (d->device_started) {
		d->ops->stop(d->dev);
		d->device_started = 0;
	}
	if (d->writer_started) {
		plat_mutex_lock(&d->q_lock);
		d->writer_stop = 1;
		plat_cond_signal(&d->q_cond);
		plat_mutex_unlock(&d->q_lock);
		plat_thread_join(d->writer_th);
		d->writer_started = 0;
	}
	if (d->device_open || d->have_start) {
		status_line(d);
		log_event("exit", "frames=%llu dropped=%llu overruns=%llu code=%d",
		          (unsigned long long)d->frames_out, (unsigned long long)d->frames_dropped,
		          (unsigned long long)d->overruns, d->exit_code);
	}
	if (d->device_open) {
		d->ops->close(d->dev);
		d->device_open = 0;
	}
	if (d->out)
		plat_out_close(d->out);
	for (i = 0; i < d->n; i++) {
		atkdaq_ring_free(&d->ch[i].ring);
		free(d->cap_raw[i]);
		free(d->cap_f[i]);
	}
	for (i = 0; i < d->pool_n; i++)
		free(d->pool[i]);
	/* the stdin thread may be blocked in a read; it is not joined, and the
	 * process exit ends it */
	free(d);
}
