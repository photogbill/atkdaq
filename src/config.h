/*
 * config.h — everything atkdaq can be told, in one struct.
 *
 * Read from a key = value file (--config) and then from --set key=value on
 * the command line, in that order, so a command line always wins. Unknown
 * keys are REFUSED with the line number: a misspelt key that is silently
 * ignored is a setting the operator believes is in force and is not.
 * `atkdaq help-config` prints every key with its default and meaning.
 */
#ifndef ATKDAQ_CONFIG_H
#define ATKDAQ_CONFIG_H

#include <stdint.h>

#include "atkdaq_frame.h"

#define ATKDAQ_MAX_EMITTERS 4
#define ATKDAQ_MAX_SYNTH_DROPS 8

typedef struct synth_emitter {
	double f_offset_hz;                         /* baseband offset from fc */
	double level_db;                            /* dB re full scale, per channel */
	double steer_deg[ATKDAQ_MAX_CHANNELS];      /* ON-AIR phase at each element */
	double burst_on_s, burst_off_s;             /* 0,0 = continuous */
	int    active;
} synth_emitter;

typedef struct synth_drop {
	int    ch;
	double at_s;                                /* stream time */
	long   samples;
} synth_drop;

typedef struct atkdaq_config {
	/* device */
	char     device[16];                        /* "kraken" | "synth" */
	int      n_channels;
	char     serial[ATKDAQ_MAX_CHANNELS][16];
	uint64_t fc_hz;
	uint32_t fs_hz;
	int      gain_tenths;
	int      noise_gpio;                        /* Kraken: GPIO on channel 0's chip */
	int      bias_tee;                          /* 0 off (the only coherent-safe default) */
	int      transfer_bytes;                    /* per USB transfer, multiple of 512 */
	int      transfer_count;                    /* async buffers per device */

	/* framing and buffering */
	int      frame_ms;
	double   ring_seconds;
	int      out_queue_frames;
	int      clip_flag_thresh;                  /* clipped samples per channel per frame */
	char     out_path[512];                     /* "" = stdout */
	double   run_seconds;                       /* 0 = until quit / EOF */
	long long max_frames;                       /* 0 = unlimited */

	/* sync / cal */
	int      mode;                              /* ATKDAQ_MODE_* */
	int      settle_ms;                         /* after a noise switch */
	int      retune_settle_ms;                  /* after tune / gain */
	int      sync_samples;
	int      sync_max_lag;
	int      sync_verify;                       /* 1: two captures must agree to the sample */
	double   sync_min_ptn_db;                   /* refuse a sync weaker than this */
	int      cal_samples;
	double   cal_max_spread_deg;                /* refuse a cal worse than this */
	int      sync_check_static_s;
	int      sync_check_mobile_s;
	int      cal_interval_static_s;
	int      cal_interval_mobile_s;
	int      retune_policy;                     /* 0 auto (§6), 1 always sync+cal, 2 cal only */
	int      drop_thresh_us;
	int      drop_persist;
	int      auto_start_cal;                    /* 1: sync + cal at start (the default) */

	/* synthetic device */
	unsigned synth_seed;
	int      synth_fast;                        /* 1: as fast as the consumer reads */
	long     synth_start[ATKDAQ_MAX_CHANNELS];  /* stream start offsets, samples */
	double   synth_frac[ATKDAQ_MAX_CHANNELS];   /* receiver fractional delay, samples */
	double   synth_gain_db[ATKDAQ_MAX_CHANNELS];
	double   synth_phase_deg[ATKDAQ_MAX_CHANNELS];
	double   synth_noise_db;                    /* noise source level, dBFS per channel */
	double   synth_floor_db;                    /* receiver noise, dBFS per channel */
	double   synth_jitter_us;                   /* extra USB arrival latency, uniform */
	int      synth_retune_moves_delays;         /* 1: the §6 hypothesis is FALSE */
	synth_emitter synth_emitter[ATKDAQ_MAX_EMITTERS];
	synth_drop synth_drop[ATKDAQ_MAX_SYNTH_DROPS];
	int      synth_n_drops;
} atkdaq_config;

void atkdaq_config_defaults(atkdaq_config *c);
/* Apply one "key = value" (also "key=value"). Returns 0, or -1 unknown key,
 * -2 bad value; err gets a sentence. */
int  atkdaq_config_set(atkdaq_config *c, const char *key, const char *value,
                       char *err, int errlen);
/* Load a file. Returns 0, or -1 with err naming the file and line. */
int  atkdaq_config_load(atkdaq_config *c, const char *path, char *err, int errlen);
/* Cross-field checks (channel counts, ranges); -1 with a sentence. */
int  atkdaq_config_validate(const atkdaq_config *c, char *err, int errlen);
void atkdaq_config_help(void);

#endif /* ATKDAQ_CONFIG_H */
