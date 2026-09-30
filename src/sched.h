/*
 * sched.h — the calibration policy: WHEN to sync, calibrate or check.
 *
 * Pure bookkeeping: it is told what happened (start, retune, gain change,
 * misalignment, a consumer's request, the passage of stream time) and asked
 * what to do next. daq.c does the doing. Keeping the decisions here, with
 * no device and no thread, is what lets tests/test_smoke.c walk the policy
 * through its cases directly.
 *
 *   event                 static                          mobile
 *   start                 sync + cal                      sync + cal
 *   retune                cal only once retunes have been seen to keep the
 *                         delays (§6), else sync + cal    same
 *   gain change           cal                             cal
 *   misaligned            sync + cal at the next `quiet`,  sync + cal now
 *                         or after quiet_wait_s anyway
 *   periodic check        every sync_check_static_s       every sync_check_mobile_s
 *   periodic cal          every cal_interval_static_s     every cal_interval_mobile_s
 *   consumer sync/cal/check  now                          now
 *
 * The policy lives in the DAQ so that a consumer that has crashed leaves a
 * coherent DAQ behind; the consumer can override every interval.
 *
 * THE FAST-RETUNE HYPOTHESIS IS LEARNED, NOT ASSUMED. Until retunes have
 * been observed to preserve the integer delays `hyp_needed` times in a row,
 * every retune gets a full sync, and the sync's result is the observation.
 * One retune that moves a delay refutes it for the session. `atkdaq probe
 * --retune-test` measures the same thing deliberately.
 */
#ifndef ATKDAQ_SCHED_H
#define ATKDAQ_SCHED_H

#include <stdint.h>

#include "config.h"

typedef enum { PROC_NONE = 0, PROC_FULL, PROC_CAL, PROC_CHECK } proc_kind;

#define SCHED_P_FULL  1u
#define SCHED_P_CAL   2u
#define SCHED_P_CHECK 4u

typedef struct sched {
	int mode;
	int sync_check_s[2];
	int cal_interval_s[2];
	int retune_policy;
	int quiet_wait_s;
	int hyp;                  /* >0 confirmations, <0 refuted, 0 unknown */
	int hyp_needed;
	int64_t last_delays_ns;   /* stream time of the last delay measurement */
	int64_t last_cal_ns;
	unsigned pending;
	int after_retune;         /* the pending FULL follows a retune */
	int cal_after_retune;     /* the pending CAL is a cal-only retune */
	int misaligned;
	int64_t misaligned_ns;
	int quiet;
	int noise_held;
	int calibrated_once;
	int fail_count;
	int64_t retry_ns;
} sched;

void sched_init(sched *s, const atkdaq_config *c);
void sched_on_start(sched *s, int64_t now, int auto_cal);
void sched_on_retune(sched *s, int64_t now);
void sched_on_gain(sched *s, int64_t now);
void sched_on_misaligned(sched *s, int64_t now);
void sched_on_quiet(sched *s);
void sched_request(sched *s, proc_kind k);
void sched_set_mode(sched *s, int mode);
void sched_set_interval(sched *s, int what, int seconds);
void sched_hold_noise(sched *s, int held);
/* A calibration found a whole-sample residual: the integer delays moved
 * under it. After a cal-only retune that refutes the §6 hypothesis; either
 * way a sync is due now, with no failure back-off. */
void sched_on_delays_moved(sched *s);
/* What to start now; PROC_NONE when nothing is due. */
proc_kind sched_next(sched *s, int64_t now);
/* A procedure finished (ok or not; delays_changed = the integer delays moved). */
void sched_done(sched *s, proc_kind k, int64_t now, int ok, int delays_changed);
const char *sched_proc_name(proc_kind k);

#endif
