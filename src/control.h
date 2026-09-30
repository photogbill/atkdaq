/*
 * control.h — the commands a consumer sends on atkdaq's stdin, one per line.
 *
 *     tune <hz>             retune every channel (446e6 and 446000000 both work)
 *     gain <tenths_db>      identical manual gain on every channel
 *     sync                  measure delays and weights now (noise source)
 *     cal                   measure weights only (noise source)
 *     check                 re-measure delays; change them only if two captures agree
 *     noise on|off|auto     hold the noise source on/off (T1-style bench work);
 *                           auto hands it back to the policy
 *     mode static|mobile    the calibration policy (sched.c)
 *     quiet                 "now is a good moment" - a pending static-mode resync runs
 *     interval <what> <s>   sync_check | cal : override the policy's timer
 *     status                one status line now
 *     quit                  stop cleanly (EOF on stdin does the same)
 *
 * A malformed line is answered with event=error on stderr and otherwise
 * ignored: a consumer's typo must not stop the acquisition.
 */
#ifndef ATKDAQ_CONTROL_H
#define ATKDAQ_CONTROL_H

#include <stdint.h>

typedef enum {
	CMD_NONE = 0, CMD_TUNE, CMD_GAIN, CMD_SYNC, CMD_CAL, CMD_CHECK, CMD_NOISE,
	CMD_MODE, CMD_QUIET, CMD_INTERVAL, CMD_STATUS, CMD_QUIT
} cmd_kind;

typedef struct command {
	cmd_kind kind;
	int64_t ival;          /* tune: Hz; gain: tenths; noise: 0 off 1 on 2 auto; mode;
	                          interval: seconds */
	int what;              /* interval: 0 sync_check, 1 cal */
} command;

/* Returns 0 with *out filled, 1 for a blank line or comment, -1 with err. */
int control_parse(const char *line, command *out, char *err, int errlen);

#endif
