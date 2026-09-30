/* sched.c — see sched.h. */
#include "sched.h"

#include <string.h>

#include "atkdaq_frame.h"

#define NS 1000000000LL

void sched_init(sched *s, const atkdaq_config *c)
{
	memset(s, 0, sizeof(*s));
	s->mode = c->mode;
	s->sync_check_s[ATKDAQ_MODE_STATIC] = c->sync_check_static_s;
	s->sync_check_s[ATKDAQ_MODE_MOBILE] = c->sync_check_mobile_s;
	s->cal_interval_s[ATKDAQ_MODE_STATIC] = c->cal_interval_static_s;
	s->cal_interval_s[ATKDAQ_MODE_MOBILE] = c->cal_interval_mobile_s;
	s->retune_policy = c->retune_policy;
	s->quiet_wait_s = 5;
	s->hyp_needed = 3;
}

void sched_on_start(sched *s, int64_t now, int auto_cal)
{
	s->last_delays_ns = now;
	s->last_cal_ns = now;
	if (auto_cal)
		s->pending |= SCHED_P_FULL;
}

void sched_on_retune(sched *s, int64_t now)
{
	int cal_only;
	(void)now;
	if (s->retune_policy == 2)
		cal_only = 1;
	else if (s->retune_policy == 1)
		cal_only = 0;
	else
		cal_only = s->hyp >= s->hyp_needed;
	if (cal_only) {
		s->pending |= SCHED_P_CAL;
		s->after_retune = 0;
		s->cal_after_retune = 1;
	} else {
		s->pending |= SCHED_P_FULL;
		s->after_retune = 1;
		s->cal_after_retune = 0;
	}
}

void sched_on_gain(sched *s, int64_t now)
{
	(void)now;
	s->pending |= SCHED_P_CAL;
}

void sched_on_misaligned(sched *s, int64_t now)
{
	if (!s->misaligned) {
		s->misaligned = 1;
		s->misaligned_ns = now;
		s->quiet = 0;
	}
	s->pending |= SCHED_P_FULL;
}

void sched_on_quiet(sched *s) { s->quiet = 1; }

void sched_request(sched *s, proc_kind k)
{
	if (k == PROC_FULL)
		s->pending |= SCHED_P_FULL;
	else if (k == PROC_CAL)
		s->pending |= SCHED_P_CAL;
	else if (k == PROC_CHECK)
		s->pending |= SCHED_P_CHECK;
	/* an explicit request is not held back by an earlier failure's back-off */
	s->retry_ns = 0;
}

void sched_set_mode(sched *s, int mode) { s->mode = mode ? ATKDAQ_MODE_MOBILE : ATKDAQ_MODE_STATIC; }

void sched_set_interval(sched *s, int what, int seconds)
{
	if (what == 0)
		s->sync_check_s[s->mode] = seconds;
	else
		s->cal_interval_s[s->mode] = seconds;
}

void sched_hold_noise(sched *s, int held) { s->noise_held = held; }

void sched_on_delays_moved(sched *s)
{
	if (s->cal_after_retune && s->retune_policy == 0)
		s->hyp = -1;              /* a retune did not keep the delays after all */
	s->cal_after_retune = 0;
	s->after_retune = 0;
	s->pending &= ~SCHED_P_CAL;
	s->pending |= SCHED_P_FULL;
	s->retry_ns = 0;
}

proc_kind sched_next(sched *s, int64_t now)
{
	int chk = s->sync_check_s[s->mode], cal = s->cal_interval_s[s->mode];
	if (s->noise_held)
		return PROC_NONE;          /* the operator has the noise source */
	if (s->retry_ns && now < s->retry_ns)
		return PROC_NONE;
	/* timers only run once there is a calibration to keep fresh */
	if (s->calibrated_once) {
		if (chk > 0 && now - s->last_delays_ns >= (int64_t)chk * NS)
			s->pending |= SCHED_P_CHECK;
		if (cal > 0 && now - s->last_cal_ns >= (int64_t)cal * NS)
			s->pending |= SCHED_P_CAL;
	}
	if (s->pending & SCHED_P_FULL) {
		/* A misaligned array in static mode waits for a quiet moment the
		 * consumer names, but not for ever: misaligned data is useless, so
		 * waiting longer only prolongs the uselessness. */
		if (s->misaligned && s->mode == ATKDAQ_MODE_STATIC && s->calibrated_once && !s->quiet &&
		    now - s->misaligned_ns < (int64_t)s->quiet_wait_s * NS)
			return PROC_NONE;
		return PROC_FULL;
	}
	if (s->pending & SCHED_P_CHECK)
		return PROC_CHECK;
	if (s->pending & SCHED_P_CAL)
		return PROC_CAL;
	return PROC_NONE;
}

void sched_done(sched *s, proc_kind k, int64_t now, int ok, int delays_changed)
{
	if (!ok) {
		int64_t back = 2LL << (s->fail_count < 4 ? s->fail_count : 4);   /* 2..32 s */
		s->fail_count++;
		s->retry_ns = now + back * NS;
		return;
	}
	s->fail_count = 0;
	s->retry_ns = 0;
	switch (k) {
	case PROC_FULL:
	case PROC_CHECK:
		/* both measure delays AND weights (one noise-on period) */
		s->pending &= ~(SCHED_P_FULL | SCHED_P_CHECK | SCHED_P_CAL);
		s->last_delays_ns = now;
		s->last_cal_ns = now;
		s->misaligned = 0;
		s->quiet = 0;
		s->calibrated_once = 1;
		s->cal_after_retune = 0;
		if (k == PROC_FULL && s->after_retune) {
			if (delays_changed)
				s->hyp = -1;
			else if (s->hyp >= 0)
				s->hyp++;
			s->after_retune = 0;
		}
		break;
	case PROC_CAL:
		s->pending &= ~SCHED_P_CAL;
		s->last_cal_ns = now;
		s->calibrated_once = 1;
		s->cal_after_retune = 0;
		break;
	default:
		break;
	}
}

const char *sched_proc_name(proc_kind k)
{
	switch (k) {
	case PROC_FULL: return "sync+cal";
	case PROC_CAL: return "cal";
	case PROC_CHECK: return "check";
	default: return "none";
	}
}
