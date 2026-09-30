/* control.c — see control.h. */
#include "control.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "atkdaq_frame.h"

#ifdef _MSC_VER
#define strcasecmp _stricmp
#else
#include <strings.h>
#endif

static int word(const char **p, char *out, int cap)
{
	int n = 0;
	while (**p && isspace((unsigned char)**p))
		(*p)++;
	while (**p && !isspace((unsigned char)**p)) {
		if (n < cap - 1)
			out[n++] = **p;
		(*p)++;
	}
	out[n] = 0;
	return n;
}

static int number(const char *s, double *out)
{
	char *end;
	if (!*s)
		return -1;
	*out = strtod(s, &end);
	return (*end == 0) ? 0 : -1;
}

int control_parse(const char *line, command *out, char *err, int errlen)
{
	char w0[32], w1[64], w2[64];
	const char *p = line;
	double v;
	memset(out, 0, sizeof(*out));
	if (!word(&p, w0, sizeof(w0)) || w0[0] == '#')
		return 1;
	word(&p, w1, sizeof(w1));
	word(&p, w2, sizeof(w2));

	if (!strcasecmp(w0, "tune")) {
		if (number(w1, &v) != 0 || v < 24e6 || v > 1.766e9) {
			snprintf(err, errlen, "tune wants a frequency in Hz between 24e6 and 1766e6, got '%s'", w1);
			return -1;
		}
		out->kind = CMD_TUNE;
		out->ival = (int64_t)(v + 0.5);
		return 0;
	}
	if (!strcasecmp(w0, "gain")) {
		if (number(w1, &v) != 0 || v < 0 || v > 600) {
			snprintf(err, errlen, "gain wants tenths of a dB (e.g. 280 for 28.0 dB), got '%s'", w1);
			return -1;
		}
		out->kind = CMD_GAIN;
		out->ival = (int64_t)(v + 0.5);
		return 0;
	}
	if (!strcasecmp(w0, "sync")) { out->kind = CMD_SYNC; return 0; }
	if (!strcasecmp(w0, "cal")) { out->kind = CMD_CAL; return 0; }
	if (!strcasecmp(w0, "check")) { out->kind = CMD_CHECK; return 0; }
	if (!strcasecmp(w0, "quiet")) { out->kind = CMD_QUIET; return 0; }
	if (!strcasecmp(w0, "status")) { out->kind = CMD_STATUS; return 0; }
	if (!strcasecmp(w0, "quit") || !strcasecmp(w0, "exit")) { out->kind = CMD_QUIT; return 0; }
	if (!strcasecmp(w0, "noise")) {
		out->kind = CMD_NOISE;
		if (!strcasecmp(w1, "on")) out->ival = 1;
		else if (!strcasecmp(w1, "off")) out->ival = 0;
		else if (!strcasecmp(w1, "auto")) out->ival = 2;
		else {
			snprintf(err, errlen, "noise wants on, off or auto, got '%s'", w1);
			return -1;
		}
		return 0;
	}
	if (!strcasecmp(w0, "mode")) {
		out->kind = CMD_MODE;
		if (!strcasecmp(w1, "static")) out->ival = ATKDAQ_MODE_STATIC;
		else if (!strcasecmp(w1, "mobile")) out->ival = ATKDAQ_MODE_MOBILE;
		else {
			snprintf(err, errlen, "mode wants static or mobile, got '%s'", w1);
			return -1;
		}
		return 0;
	}
	if (!strcasecmp(w0, "interval")) {
		out->kind = CMD_INTERVAL;
		if (!strcasecmp(w1, "sync_check") || !strcasecmp(w1, "check")) out->what = 0;
		else if (!strcasecmp(w1, "cal")) out->what = 1;
		else {
			snprintf(err, errlen, "interval wants sync_check or cal, got '%s'", w1);
			return -1;
		}
		if (number(w2, &v) != 0 || v < 0 || v > 86400) {
			snprintf(err, errlen, "interval wants seconds (0 = off), got '%s'", w2);
			return -1;
		}
		out->ival = (int64_t)(v + 0.5);
		return 0;
	}
	snprintf(err, errlen, "unknown command '%s' (tune gain sync cal check noise mode quiet interval status quit)", w0);
	return -1;
}
