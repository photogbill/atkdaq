/* config.c — see config.h. */
#include "config.h"
#include "atkdaq_core.h"

#include <ctype.h>
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _MSC_VER
#define strcasecmp _stricmp
#else
#include <strings.h>
#endif

enum kt { K_INT, K_U64, K_U32, K_DBL, K_STR, K_BOOL, K_MODE, K_DEVICE,
          K_SERIALS, K_LONGLIST, K_DBLLIST, K_EMITTER, K_DROP };

typedef struct key {
	const char *name;
	enum kt type;
	size_t off;
	size_t size;          /* strings: capacity */
	const char *help;
} key;

#define F(field) offsetof(atkdaq_config, field)

static const key KEYS[] = {
	/* device */
	{"device", K_DEVICE, F(device), 16, "kraken | synth"},
	{"channels", K_INT, F(n_channels), 0, "coherent channels (5 for a KrakenSDR)"},
	{"serial", K_SERIALS, F(serial), 0,
	 "EEPROM serials in CHANNEL ORDER, comma separated (Kraken: 1000,1001,1002,1003,1004)"},
	{"fc_hz", K_U64, F(fc_hz), 0, "centre frequency, Hz"},
	{"fs_hz", K_U32, F(fs_hz), 0, "sample rate per channel, Hz"},
	{"gain_tenths_db", K_INT, F(gain_tenths), 0,
	 "manual tuner gain, tenths of dB, identical on every channel (R820T steps: 0 9 14 27 37 77 87 125 144 157 166 197 207 229 254 280 297 328 338 364 372 386 402 421 434 439 445 480 496)"},
	{"noise_gpio", K_INT, F(noise_gpio), 0,
	 "GPIO of channel 0's RTL2832U that switches the noise source (Kraken: 0)"},
	{"bias_tee", K_BOOL, F(bias_tee), 0, "per-channel bias tees on the Kraken (GPIO 1..5 of channel 0's chip)"},
	{"transfer_bytes", K_INT, F(transfer_bytes), 0, "bytes per USB transfer (multiple of 16384)"},
	{"transfer_count", K_INT, F(transfer_count), 0, "USB transfers in flight per device"},
	/* framing */
	{"frame_ms", K_INT, F(frame_ms), 0, "frame length, ms (50 -> 120000 samples at 2.4 MSPS)"},
	{"ring_seconds", K_DBL, F(ring_seconds), 0, "per-channel ring depth, seconds"},
	{"out_queue_frames", K_INT, F(out_queue_frames), 0,
	 "frames waiting for the consumer before new ones are dropped"},
	{"clip_flag_thresh", K_INT, F(clip_flag_thresh), 0,
	 "clipped samples per channel per frame that set the CLIPPED flag"},
	{"out", K_STR, F(out_path), 512, "output file ('' = stdout)"},
	{"seconds", K_DBL, F(run_seconds), 0, "stop after this long (0 = until quit or EOF on stdin)"},
	{"frames", K_LONGLIST, F(max_frames), 1, "stop after this many frames (0 = unlimited)"},
	/* sync and calibration */
	{"mode", K_MODE, F(mode), 0, "static | mobile (the calibration policy, see sched.c)"},
	{"settle_ms", K_INT, F(settle_ms), 0, "discarded after every noise-source switch"},
	{"retune_settle_ms", K_INT, F(retune_settle_ms), 0, "discarded after a retune or gain change"},
	{"sync_samples", K_INT, F(sync_samples), 0, "samples per sync capture"},
	{"sync_max_lag", K_INT, F(sync_max_lag), 0, "largest delay searched after coarse alignment, samples"},
	{"sync_verify", K_BOOL, F(sync_verify), 0, "require a second capture to agree to the sample"},
	{"sync_min_ptn_db", K_DBL, F(sync_min_ptn_db), 0, "refuse a sync whose peak-to-next is below this"},
	{"cal_samples", K_INT, F(cal_samples), 0, "samples per calibration capture"},
	{"cal_max_spread_deg", K_DBL, F(cal_max_spread_deg), 0, "refuse a calibration whose residual exceeds this"},
	{"sync_check_static_s", K_INT, F(sync_check_static_s), 0, "periodic delay check, static mode (0 = off)"},
	{"sync_check_mobile_s", K_INT, F(sync_check_mobile_s), 0, "periodic delay check, mobile mode (0 = off)"},
	{"cal_interval_static_s", K_INT, F(cal_interval_static_s), 0, "periodic recalibration, static (0 = off)"},
	{"cal_interval_mobile_s", K_INT, F(cal_interval_mobile_s), 0, "periodic recalibration, mobile (0 = off)"},
	{"retune_policy", K_INT, F(retune_policy), 0,
	 "0 auto (cal only while retunes have preserved the delays), 1 always sync+cal, 2 always cal only"},
	{"drop_thresh_us", K_INT, F(drop_thresh_us), 0, "smallest per-channel timing shift called a loss"},
	{"drop_persist", K_INT, F(drop_persist), 0, "evaluations a shift must survive before it is believed"},
	{"auto_start_cal", K_BOOL, F(auto_start_cal), 0, "sync and calibrate as soon as streaming starts"},
	/* synthetic device */
	{"synth.seed", K_U32, F(synth_seed), 0, "random seed"},
	{"synth.fast", K_BOOL, F(synth_fast), 0, "1: run as fast as the consumer reads, not in real time"},
	{"synth.start", K_LONGLIST, F(synth_start), ATKDAQ_MAX_CHANNELS,
	 "per-channel stream start offsets, samples (expected delay[i] = start[0] - start[i])"},
	{"synth.frac", K_DBLLIST, F(synth_frac), ATKDAQ_MAX_CHANNELS, "per-channel receiver fractional delay, samples"},
	{"synth.gain_db", K_DBLLIST, F(synth_gain_db), ATKDAQ_MAX_CHANNELS, "per-channel receiver gain error, dB"},
	{"synth.phase_deg", K_DBLLIST, F(synth_phase_deg), ATKDAQ_MAX_CHANNELS, "per-channel receiver phase, degrees"},
	{"synth.noise_db", K_DBL, F(synth_noise_db), 0, "noise source level, dBFS"},
	{"synth.floor_db", K_DBL, F(synth_floor_db), 0, "receiver noise floor, dBFS"},
	{"synth.jitter_us", K_DBL, F(synth_jitter_us), 0, "extra USB arrival latency, uniform 0..this"},
	{"synth.retune_moves_delays", K_BOOL, F(synth_retune_moves_delays), 0,
	 "1: a retune shifts the stream offsets (the fast-retune hypothesis is false)"},
	{"synth.emitter", K_EMITTER, 0, 0,
	 "f_offset_hz,level_dbfs,phase_ch0_deg,...,phase_chN_deg[,burst_on_s,burst_off_s] (repeatable, up to 4)"},
	{"synth.drop", K_DROP, 0, 0, "channel,at_seconds,samples (repeatable, up to 8): that channel loses samples"},
};

#define NKEYS (sizeof(KEYS) / sizeof(KEYS[0]))

void atkdaq_config_defaults(atkdaq_config *c)
{
	int i;
	memset(c, 0, sizeof(*c));
	strcpy(c->device, "kraken");
	c->n_channels = 5;
	for (i = 0; i < ATKDAQ_MAX_CHANNELS; i++)
		snprintf(c->serial[i], sizeof(c->serial[i]), "%d", 1000 + i);
	c->fc_hz = 446000000ULL;
	c->fs_hz = 2400000u;
	c->gain_tenths = 280;
	c->noise_gpio = 0;
	c->bias_tee = 0;
	c->transfer_bytes = 262144;
	c->transfer_count = 8;
	c->frame_ms = 50;
	c->ring_seconds = 4.0;
	c->out_queue_frames = 16;
	c->clip_flag_thresh = 12;
	c->mode = ATKDAQ_MODE_STATIC;
	c->settle_ms = 40;
	c->retune_settle_ms = 50;
	c->sync_samples = 131072;
	c->sync_max_lag = 32768;
	c->sync_verify = 1;
	c->sync_min_ptn_db = 10.0;
	c->cal_samples = 131072;
	c->cal_max_spread_deg = 5.0;
	c->sync_check_static_s = 300;
	c->sync_check_mobile_s = 60;
	c->cal_interval_static_s = 900;
	c->cal_interval_mobile_s = 120;
	c->retune_policy = 0;
	c->drop_thresh_us = 1000;
	c->drop_persist = 2;
	c->auto_start_cal = 1;
	c->synth_seed = 1;
	c->synth_fast = 0;
	c->synth_noise_db = -12.0;
	c->synth_floor_db = -40.0;
	c->synth_jitter_us = 300.0;
}

static const char *trim(char *s)
{
	char *e;
	while (*s && isspace((unsigned char)*s))
		s++;
	e = s + strlen(s);
	while (e > s && isspace((unsigned char)e[-1]))
		*--e = 0;
	return s;
}

static int parse_double(const char *v, double *out)
{
	char *end;
	errno = 0;
	*out = strtod(v, &end);
	if (end == v || errno != 0)
		return -1;
	while (*end && isspace((unsigned char)*end))
		end++;
	return *end ? -1 : 0;
}

static int parse_ll(const char *v, long long *out)
{
	char *end;
	double d;
	errno = 0;
	*out = strtoll(v, &end, 10);
	if (end != v && errno == 0) {
		while (*end && isspace((unsigned char)*end))
			end++;
		if (!*end)
			return 0;
	}
	/* allow 446e6 and 446000000.0 for frequencies */
	if (parse_double(v, &d) == 0 && d == (double)(long long)d) {
		*out = (long long)d;
		return 0;
	}
	return -1;
}

/* split a comma list into up to max doubles; returns count or -1 */
static int parse_dlist(const char *v, double *out, int max)
{
	char buf[1024];
	char *p, *tok;
	int n = 0;
	if (strlen(v) >= sizeof(buf))
		return -1;
	strcpy(buf, v);
	p = buf;
	for (;;) {
		char *comma = strchr(p, ',');
		if (comma)
			*comma = 0;
		tok = (char *)trim(p);
		if (*tok) {
			if (n >= max || parse_double(tok, &out[n]) != 0)
				return -1;
			n++;
		}
		if (!comma)
			break;
		p = comma + 1;
	}
	return n;
}

int atkdaq_config_set(atkdaq_config *c, const char *key_in, const char *value_in,
                      char *err, int errlen)
{
	char keybuf[128], valbuf[1024];
	const char *k, *v;
	size_t i;
	if (strlen(key_in) >= sizeof(keybuf) || strlen(value_in) >= sizeof(valbuf)) {
		snprintf(err, errlen, "setting too long");
		return -2;
	}
	strcpy(keybuf, key_in);
	strcpy(valbuf, value_in);
	k = trim(keybuf);
	v = trim(valbuf);
	for (i = 0; i < NKEYS; i++) {
		const key *e = &KEYS[i];
		char *base = (char *)c + e->off;
		long long ll;
		double d;
		double list[ATKDAQ_MAX_CHANNELS + 4];
		int n, j;
		if (strcasecmp(k, e->name) != 0)
			continue;
		switch (e->type) {
		case K_INT:
			if (parse_ll(v, &ll) != 0 || ll < -2147483647LL || ll > 2147483647LL)
				goto bad;
			*(int *)base = (int)ll;
			return 0;
		case K_U32:
			if (parse_ll(v, &ll) != 0 || ll < 0 || ll > 4294967295LL)
				goto bad;
			*(uint32_t *)base = (uint32_t)ll;
			return 0;
		case K_U64:
			if (parse_ll(v, &ll) != 0 || ll < 0)
				goto bad;
			*(uint64_t *)base = (uint64_t)ll;
			return 0;
		case K_DBL:
			if (parse_double(v, &d) != 0)
				goto bad;
			*(double *)base = d;
			return 0;
		case K_STR:
			if (strlen(v) >= e->size)
				goto bad;
			strcpy(base, v);
			return 0;
		case K_BOOL:
			if (!strcasecmp(v, "1") || !strcasecmp(v, "on") || !strcasecmp(v, "yes") ||
			    !strcasecmp(v, "true"))
				*(int *)base = 1;
			else if (!strcasecmp(v, "0") || !strcasecmp(v, "off") || !strcasecmp(v, "no") ||
			         !strcasecmp(v, "false"))
				*(int *)base = 0;
			else
				goto bad;
			return 0;
		case K_MODE:
			if (!strcasecmp(v, "static"))
				*(int *)base = ATKDAQ_MODE_STATIC;
			else if (!strcasecmp(v, "mobile"))
				*(int *)base = ATKDAQ_MODE_MOBILE;
			else
				goto bad;
			return 0;
		case K_DEVICE:
			if (strcasecmp(v, "kraken") && strcasecmp(v, "synth"))
				goto bad;
			snprintf(base, e->size, "%s", v);
			for (j = 0; base[j]; j++)
				base[j] = (char)tolower((unsigned char)base[j]);
			return 0;
		case K_SERIALS: {
			char tmp[1024], *p = tmp;
			strcpy(tmp, v);
			for (j = 0; j < ATKDAQ_MAX_CHANNELS; j++) {
				char *comma = strchr(p, ',');
				const char *s;
				if (comma)
					*comma = 0;
				s = trim(p);
				if (strlen(s) >= sizeof(c->serial[0]))
					goto bad;
				strcpy(c->serial[j], s);
				if (!comma) {
					j++;
					break;
				}
				p = comma + 1;
			}
			for (; j < ATKDAQ_MAX_CHANNELS; j++)
				c->serial[j][0] = 0;
			return 0;
		}
		case K_LONGLIST:
			n = parse_dlist(v, list, (int)(e->size ? e->size : 1));
			if (n < 0)
				goto bad;
			if (e->size == 1) {
				*(long long *)base = n > 0 ? (long long)list[0] : 0;
			} else {
				for (j = 0; j < (int)e->size; j++)
					((long *)base)[j] = j < n ? (long)list[j] : 0;
			}
			return 0;
		case K_DBLLIST:
			n = parse_dlist(v, list, (int)e->size);
			if (n < 0)
				goto bad;
			for (j = 0; j < (int)e->size; j++)
				((double *)base)[j] = j < n ? list[j] : 0.0;
			return 0;
		case K_EMITTER: {
			synth_emitter *em = NULL;
			int nch = c->n_channels;
			for (j = 0; j < ATKDAQ_MAX_EMITTERS; j++)
				if (!c->synth_emitter[j].active) {
					em = &c->synth_emitter[j];
					break;
				}
			if (!em) {
				snprintf(err, errlen, "synth.emitter: at most %d emitters", ATKDAQ_MAX_EMITTERS);
				return -2;
			}
			n = parse_dlist(v, list, ATKDAQ_MAX_CHANNELS + 4);
			if (n != 2 + nch && n != 4 + nch) {
				snprintf(err, errlen,
				         "synth.emitter wants f_offset_hz,level_dbfs and %d phases "
				         "(one per channel, set 'channels' first), optionally burst_on_s,burst_off_s; got %d numbers",
				         nch, n);
				return -2;
			}
			memset(em, 0, sizeof(*em));
			em->f_offset_hz = list[0];
			em->level_db = list[1];
			for (j = 0; j < nch; j++)
				em->steer_deg[j] = list[2 + j];
			if (n == 4 + nch) {
				em->burst_on_s = list[2 + nch];
				em->burst_off_s = list[3 + nch];
			}
			em->active = 1;
			return 0;
		}
		case K_DROP: {
			synth_drop *dr;
			if (c->synth_n_drops >= ATKDAQ_MAX_SYNTH_DROPS) {
				snprintf(err, errlen, "synth.drop: at most %d", ATKDAQ_MAX_SYNTH_DROPS);
				return -2;
			}
			n = parse_dlist(v, list, 3);
			if (n != 3 || list[2] < 1)
				goto bad;
			dr = &c->synth_drop[c->synth_n_drops++];
			dr->ch = (int)list[0];
			dr->at_s = list[1];
			dr->samples = (long)list[2];
			return 0;
		}
		}
	bad:
		snprintf(err, errlen, "bad value for %s: '%s' (%s)", e->name, v, e->help);
		return -2;
	}
	snprintf(err, errlen, "unknown setting '%s' - `atkdaq help-config` lists every key", k);
	return -1;
}

int atkdaq_config_load(atkdaq_config *c, const char *path, char *err, int errlen)
{
	FILE *f = fopen(path, "r");
	char line[1024], e2[512];
	int lineno = 0;
	if (!f) {
		snprintf(err, errlen, "cannot open config file %s", path);
		return -1;
	}
	while (fgets(line, sizeof(line), f)) {
		char *hash, *eq;
		const char *k;
		lineno++;
		hash = strchr(line, '#');
		if (hash)
			*hash = 0;
		k = trim(line);
		if (!*k)
			continue;
		eq = strchr((char *)k, '=');
		if (!eq) {
			snprintf(err, errlen, "%s:%d: expected key = value", path, lineno);
			fclose(f);
			return -1;
		}
		*eq = 0;
		if (atkdaq_config_set(c, k, eq + 1, e2, sizeof(e2)) != 0) {
			snprintf(err, errlen, "%s:%d: %s", path, lineno, e2);
			fclose(f);
			return -1;
		}
	}
	fclose(f);
	return 0;
}

int atkdaq_config_validate(const atkdaq_config *c, char *err, int errlen)
{
	int i;
	if (c->n_channels < 1 || c->n_channels > ATKDAQ_MAX_CHANNELS) {
		snprintf(err, errlen, "channels must be 1..%d", ATKDAQ_MAX_CHANNELS);
		return -1;
	}
	if (c->fs_hz < 225001 || c->fs_hz > 3200000) {
		snprintf(err, errlen, "fs_hz %u is outside what an RTL2832U can do (225001..3200000)", c->fs_hz);
		return -1;
	}
	if (c->frame_ms < 5 || c->frame_ms > 1000) {
		snprintf(err, errlen, "frame_ms must be 5..1000");
		return -1;
	}
	if ((uint64_t)c->fs_hz * (uint64_t)c->frame_ms % 1000u != 0) {
		snprintf(err, errlen, "fs_hz x frame_ms must be a whole number of samples");
		return -1;
	}
	if (c->transfer_bytes < 16384 || c->transfer_bytes % 16384 != 0) {
		snprintf(err, errlen, "transfer_bytes must be a multiple of 16384 (librtlsdr's URB size)");
		return -1;
	}
	if (c->ring_seconds < 0.5 || c->ring_seconds > 60) {
		snprintf(err, errlen, "ring_seconds must be 0.5..60");
		return -1;
	}
	if (c->sync_samples < 8192 || c->cal_samples < 8 * ATKDAQ_CAL_NFFT) {
		snprintf(err, errlen, "sync_samples >= 8192 and cal_samples >= %d", 8 * ATKDAQ_CAL_NFFT);
		return -1;
	}
	if (c->sync_max_lag < 16 || c->sync_max_lag >= c->sync_samples) {
		snprintf(err, errlen, "sync_max_lag must be 16..sync_samples-1");
		return -1;
	}
	if (c->out_queue_frames < 2 || c->out_queue_frames > 256) {
		snprintf(err, errlen, "out_queue_frames must be 2..256");
		return -1;
	}
	if (!strcmp(c->device, "kraken")) {
		for (i = 0; i < c->n_channels; i++)
			if (!c->serial[i][0]) {
				snprintf(err, errlen,
				         "channel %d has no serial - 'serial' must list one per channel, in channel order", i);
				return -1;
			}
	}
	for (i = 0; i < c->synth_n_drops; i++)
		if (c->synth_drop[i].ch < 0 || c->synth_drop[i].ch >= c->n_channels) {
			snprintf(err, errlen, "synth.drop names channel %d, which does not exist", c->synth_drop[i].ch);
			return -1;
		}
	return 0;
}

void atkdaq_config_help(void)
{
	size_t i;
	atkdaq_config d;
	atkdaq_config_defaults(&d);
	printf("atkdaq settings (config file 'key = value', or --set key=value):\n\n");
	for (i = 0; i < NKEYS; i++)
		printf("  %-28s %s\n", KEYS[i].name, KEYS[i].help);
	printf("\nDefaults: device=%s channels=%d serial=%s..%s fc_hz=%llu fs_hz=%u gain_tenths_db=%d "
	       "frame_ms=%d mode=%s\n",
	       d.device, d.n_channels, d.serial[0], d.serial[d.n_channels - 1],
	       (unsigned long long)d.fc_hz, d.fs_hz, d.gain_tenths, d.frame_ms,
	       d.mode == ATKDAQ_MODE_STATIC ? "static" : "mobile");
}
