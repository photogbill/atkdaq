/*
 * daq.h — the acquisition engine: device -> rings -> aligned frames -> out.
 *
 * One assembler thread (the caller of daq_loop) owns everything except the
 * rings' write side (device reader threads), the output queue's drain side
 * (the writer thread) and the command queue's fill side (the stdin thread).
 * See daq.c for the whole story; main.c and probe.c only drive it.
 */
#ifndef ATKDAQ_DAQ_H
#define ATKDAQ_DAQ_H

#include <stdint.h>

#include "atkdaq_core.h"
#include "config.h"
#include "sched.h"

typedef struct daq daq;

/* The outcome of the most recent noise-source procedure (for the probe). */
typedef struct daq_result {
	proc_kind kind;
	int ok;
	int delays_changed;
	int32_t delay[ATKDAQ_MAX_CHANNELS];
	float frac[ATKDAQ_MAX_CHANNELS];
	float w_re[ATKDAQ_MAX_CHANNELS], w_im[ATKDAQ_MAX_CHANNELS];
	float coherence[ATKDAQ_MAX_CHANNELS];
	float ptn_db;
	float spread_deg;
	char why[256];
	uint32_t passport;
	int64_t t_stream_ns;
	int count;                       /* procedures finished so far */
} daq_result;

typedef struct daq_stats {
	uint64_t frames_out, frames_dropped, overruns;
	double rate_sps[ATKDAQ_MAX_CHANNELS];   /* measured over the run */
	uint64_t rx_samples[ATKDAQ_MAX_CHANNELS];
	int misaligned;
	uint32_t drops[ATKDAQ_MAX_CHANNELS];
	double stream_seconds;
	uint32_t clip_max_frame[ATKDAQ_MAX_CHANNELS];
} daq_stats;

/* out: NULL = stdout; "-" = stdout; "null" = discard (probe); else a file. */
daq *daq_create(const atkdaq_config *cfg, const char *out, int read_stdin, char *err, int errlen);
/* Open the device and start streaming. Returns 0 or an exit code (see main.c). */
int  daq_start(daq *d, char *err, int errlen);
/* Run until quit, a limit in cfg, stop_fn returning nonzero, or a fault.
 * Returns the exit code. stop_fn may issue requests (daq_request etc.). */
int  daq_loop(daq *d, int (*stop_fn)(daq *d, void *ctx), void *ctx);
void daq_destroy(daq *d);

/* Programmatic control (the probe; stdin commands go through the same). */
void daq_request(daq *d, proc_kind k);
int  daq_tune(daq *d, uint64_t fc_hz);
int  daq_busy(daq *d);                        /* a procedure or retune is in flight */
const daq_result *daq_last_result(daq *d);
void daq_get_stats(daq *d, daq_stats *s);
int64_t daq_stream_ns(daq *d);                /* stream time since start */
void daq_describe_device(daq *d, char *buf, int len);
void daq_hold_noise(daq *d, int state);       /* 0 off, 1 on, 2 auto */

#endif
