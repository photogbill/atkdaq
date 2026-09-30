/*
 * main.c — atkdaq's command line.
 *
 *   atkdaq [run] [options]            stream frames to stdout (ATK's use)
 *   atkdaq record FILE.atkq [options] the same stream into a file (fixtures)
 *   atkdaq play FILE.atkq [--loop] [--fast] [--speed X]
 *                                     replay a recording on stdout
 *   atkdaq probe [options] [--seconds N] [--retune-test] [--t1 MINUTES]
 *                                     one-screen hardware report
 *   atkdaq enum                       list RTL devices and their serials
 *   atkdaq help-config                every setting, with its meaning
 *   atkdaq version
 *
 * Options (every subcommand that opens a device):
 *   --config FILE        settings file, "key = value" (see help-config)
 *   --set KEY=VALUE      one setting; repeatable; applied after --config
 *   --device kraken|synth, --fc HZ, --gain TENTHS, --mode static|mobile,
 *   --frames N, --seconds S, --out FILE, --no-stdin, --verbose
 *
 * EXIT CODES — ATK reads these:
 *   0  normal end (quit, EOF on stdin, a limit reached, or the consumer
 *      closed the pipe)
 *   2  usage or configuration error (the message names the setting)
 *   3  device missing or could not be opened (the message names the serial)
 *   4  device opened but would not start streaming
 *   5  device fault while streaming (unplugged, USB error)
 *   7  internal error (out of memory, a thread would not start)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "daq.h"
#include "devices/device.h"
#include "log.h"
#include "plat.h"
#include "version.h"

int atkdaq_probe_main(atkdaq_config *cfg, double seconds, int retune_test, double t1_minutes);
int atkdaq_play_main(const char *path, int loop, int fast, double speed);

static void usage(void)
{
	fprintf(stderr,
	        "atkdaq %s - coherent multi-channel acquisition for the Analyst Toolkit (GPL-2.0)\n\n"
	        "  atkdaq [run] [options]             stream frames to stdout\n"
	        "  atkdaq record FILE.atkq [options]  stream into a file (use --seconds or --frames)\n"
	        "  atkdaq play FILE.atkq [--loop] [--fast] [--speed X]\n"
	        "  atkdaq probe [options] [--seconds N] [--retune-test] [--t1 MINUTES]\n"
	        "  atkdaq enum | help-config | version\n\n"
	        "options: --config FILE  --set KEY=VALUE  --device kraken|synth  --fc HZ  --gain TENTHS\n"
	        "         --mode static|mobile  --frames N  --seconds S  --out FILE  --no-stdin  --verbose\n\n"
	        "Frames go to stdout; status lines (\"ATKDAQ event=...\") go to stderr.\n"
	        "Commands on stdin: tune gain sync cal check noise mode quiet interval status quit.\n",
	        ATKDAQ_VERSION_STRING);
}

typedef struct cli {
	const char *sub;
	const char *file;          /* record/play */
	int no_stdin;
	int loop, fast;
	double speed;
	double probe_seconds, t1_minutes;
	int retune_test;
} cli;

/* Apply --config/--set/shortcuts in order. Returns 0 or 2. */
static int parse_args(int argc, char **argv, atkdaq_config *cfg, cli *c)
{
	char err[512];
	int i;
	int have_config = 0;
	/* pass 1: --config first, so --set always wins whatever the order typed */
	for (i = 1; i < argc; i++)
		if (!strcmp(argv[i], "--config") && i + 1 < argc) {
			if (atkdaq_config_load(cfg, argv[i + 1], err, sizeof(err)) != 0) {
				log_event("error", "msg=\"%s\"", err);
				return 2;
			}
			have_config = 1;
			i++;
		}
	(void)have_config;
	for (i = 1; i < argc; i++) {
		const char *a = argv[i];
		const char *v = i + 1 < argc ? argv[i + 1] : NULL;
		const char *key = NULL;
		if (!strcmp(a, "--config")) {
			i++;
			continue;
		}
		if (!strcmp(a, "--no-stdin")) { c->no_stdin = 1; continue; }
		if (!strcmp(a, "--verbose")) { log_verbose = 1; continue; }
		if (!strcmp(a, "--loop")) { c->loop = 1; continue; }
		if (!strcmp(a, "--fast")) { c->fast = 1; continue; }
		if (!strcmp(a, "--retune-test")) { c->retune_test = 1; continue; }
		if (!strcmp(a, "--speed") && v) { c->speed = atof(v); i++; continue; }
		if (!strcmp(a, "--t1") && v) { c->t1_minutes = atof(v); i++; continue; }
		if (!strcmp(a, "--set") && v) {
			char kv[1024], *eq;
			snprintf(kv, sizeof(kv), "%s", v);
			eq = strchr(kv, '=');
			if (!eq) {
				log_event("error", "msg=\"--set wants KEY=VALUE, got '%s'\"", v);
				return 2;
			}
			*eq = 0;
			if (atkdaq_config_set(cfg, kv, eq + 1, err, sizeof(err)) != 0) {
				log_event("error", "msg=\"%s\"", err);
				return 2;
			}
			i++;
			continue;
		}
		if (!strcmp(a, "--device")) key = "device";
		else if (!strcmp(a, "--fc")) key = "fc_hz";
		else if (!strcmp(a, "--gain")) key = "gain_tenths_db";
		else if (!strcmp(a, "--mode")) key = "mode";
		else if (!strcmp(a, "--frames")) key = "frames";
		else if (!strcmp(a, "--out")) key = "out";
		if (key && v) {
			if (atkdaq_config_set(cfg, key, v, err, sizeof(err)) != 0) {
				log_event("error", "msg=\"%s\"", err);
				return 2;
			}
			i++;
			continue;
		}
		if (!strcmp(a, "--seconds") && v) {
			/* the probe's own duration, or the run limit */
			c->probe_seconds = atof(v);
			if (atkdaq_config_set(cfg, "seconds", v, err, sizeof(err)) != 0) {
				log_event("error", "msg=\"%s\"", err);
				return 2;
			}
			i++;
			continue;
		}
		if (a[0] == '-' ) {
			log_event("error", "msg=\"unknown option %s (atkdaq --help)\"", a);
			return 2;
		}
		/* positional */
		if (!c->sub)
			c->sub = a;
		else if (!c->file)
			c->file = a;
		else {
			log_event("error", "msg=\"unexpected argument %s\"", a);
			return 2;
		}
	}
	return 0;
}

int main(int argc, char **argv)
{
	atkdaq_config cfg;
	cli c;
	char err[1024];
	int rc, i;
	daq *d;

	log_init();
	plat_ignore_sigpipe();
	for (i = 1; i < argc; i++)
		if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h") || !strcmp(argv[i], "/?")) {
			usage();
			return 0;
		}
	atkdaq_config_defaults(&cfg);
	memset(&c, 0, sizeof(c));
	c.speed = 1.0;
	rc = parse_args(argc, argv, &cfg, &c);
	if (rc != 0)
		return rc;
	if (!c.sub)
		c.sub = "run";

	if (!strcmp(c.sub, "version")) {
		printf("atkdaq %s (wire format v%d, core ABI %d)\n", ATKDAQ_VERSION_STRING, ATKDAQ_VERSION,
		       atkdaq_core_abi());
		return 0;
	}
	if (!strcmp(c.sub, "help-config")) {
		atkdaq_config_help();
		return 0;
	}
	if (!strcmp(c.sub, "enum")) {
		int n = atkdaq_kraken_enumerate();
		return n < 0 ? 3 : 0;
	}
	if (!strcmp(c.sub, "play")) {
		if (!c.file) {
			log_event("error", "msg=\"play wants a .atkq file\"");
			return 2;
		}
		return atkdaq_play_main(c.file, c.loop, c.fast, c.speed);
	}
	if (atkdaq_config_validate(&cfg, err, sizeof(err)) != 0) {
		log_event("error", "msg=\"%s\"", err);
		return 2;
	}
	if (!strcmp(c.sub, "probe"))
		return atkdaq_probe_main(&cfg, c.probe_seconds > 0 ? c.probe_seconds : 10.0, c.retune_test,
		                         c.t1_minutes);
	if (!strcmp(c.sub, "record")) {
		if (!c.file) {
			log_event("error", "msg=\"record wants an output file, e.g. atkdaq record T1.atkq --seconds 10\"");
			return 2;
		}
		if (cfg.run_seconds <= 0 && cfg.max_frames <= 0) {
			log_event("error", "msg=\"record needs --seconds or --frames, or it would never stop\"");
			return 2;
		}
		snprintf(cfg.out_path, sizeof(cfg.out_path), "%s", c.file);
		c.no_stdin = 1;
	} else if (strcmp(c.sub, "run") != 0) {
		log_event("error", "msg=\"unknown subcommand %s (atkdaq --help)\"", c.sub);
		return 2;
	}

	d = daq_create(&cfg, cfg.out_path[0] ? cfg.out_path : NULL, !c.no_stdin, err, sizeof(err));
	if (!d) {
		log_event("error", "msg=\"%s\"", err);
		return 7;
	}
	log_event("start", "version=%s wire=%d subcommand=%s", ATKDAQ_VERSION_STRING, ATKDAQ_VERSION, c.sub);
	rc = daq_start(d, err, sizeof(err));
	if (rc != 0) {
		log_event("error", "msg=\"%s\"", err);
		daq_destroy(d);
		return rc;
	}
	rc = daq_loop(d, NULL, NULL);
	if (rc == 6)
		rc = 0;       /* the consumer closed the pipe: a normal way to stop us */
	daq_destroy(d);
	return rc;
}
