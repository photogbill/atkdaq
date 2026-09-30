/*
 * test_smoke.c — the C-only proof that atkdaq's core is sane. No device, no
 * Python, no threads. build.bat and build.sh run it after every build; exit
 * 0 means every check passed.
 *
 * Known answers throughout: a planted delay, gain, phase or loss, and the
 * code must return it. tests/test_cross_check.py then holds the same
 * functions to their numpy twins, and tests/test_end_to_end.py runs the
 * whole program against the synthetic device.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "atkdaq_core.h"
#include "config.h"
#include "control.h"
#include "pocketfft.h"
#include "sched.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int g_pass, g_fail;

static void check(const char *name, int ok, const char *detail)
{
	if (ok) {
		g_pass++;
	} else {
		g_fail++;
		printf("  FAIL  %s  %s\n", name, detail ? detail : "");
	}
}

/* deterministic uniform (0,1] */
static unsigned long long g_rng = 0x1234567ULL;
static double urand(void)
{
	g_rng = g_rng * 6364136223846793005ULL + 1442695040888963407ULL;
	return ((double)(g_rng >> 11) + 1.0) / 9007199254740992.0;
}

/* complex Gaussian noise into n complex doubles */
static void cnoise(double *x, int n, double var)
{
	int k;
	for (k = 0; k < n; k++) {
		double r = sqrt(-var * log(urand())), t = 2 * M_PI * urand();
		x[2 * k] = r * cos(t);
		x[2 * k + 1] = r * sin(t);
	}
}

/* ------------------------------------------------------------------------ */

static void test_crc_and_header(void)
{
	uint8_t buf[ATKDAQ_HDR_BYTES(ATKDAQ_MAX_CHANNELS)];
	atkdaq_hdr h, h2;
	atkdaq_ch ch[5], ch2[5];
	size_t n;
	int i, rc;
	char d[160];
	check("crc32 of '123456789' is CBF43926", atkdaq_crc32(0, "123456789", 9) == 0xCBF43926u, NULL);

	memset(&h, 0, sizeof(h));
	memset(ch, 0, sizeof(ch));
	h.fmt = ATKDAQ_FMT_CI8;
	h.cal_state = ATKDAQ_CAL_CALIBRATED;
	h.device_kind = ATKDAQ_DEV_SYNTH;
	h.flags = ATKDAQ_F_NOISE_ON | ATKDAQ_F_SYNTHETIC;
	h.seq = 123456;
	h.t_utc_ns = 1790000000123456789LL;
	h.fc_hz = 446000000ULL;
	h.fs_hz = 2400000;
	h.samples_per_ch = 120000;
	h.segment = 7;
	h.passport = 9;
	h.cal_age_ms = 4321;
	h.spread_cdeg = 57;
	h.sync_conf = 3812;
	h.gain_tenths_db = 280;
	h.mode = ATKDAQ_MODE_MOBILE;
	h.stream_sample0 = 987654321ULL;
	h.seg_sample0 = 12345ULL;
	for (i = 0; i < 5; i++) {
		ch[i].delay = i * 7 - 3;
		ch[i].frac_delay = 0.125f * i;
		ch[i].w_re = 1.0f - 0.1f * i;
		ch[i].w_im = 0.05f * i;
		ch[i].drops = (uint32_t)i;
		ch[i].clip = (uint32_t)(10 * i);
		ch[i].sample_count = 1000000ULL * i;
	}
	n = atkdaq_hdr_pack(buf, sizeof(buf), &h, ch, 5);
	check("5-channel header is 256 bytes", n == 256, NULL);
	rc = atkdaq_hdr_unpack(buf, n, &h2, ch2, 5);
	snprintf(d, sizeof(d), "rc=%d", rc);
	check("header round-trips", rc == 256, d);
	check("fields survive", h2.seq == h.seq && h2.t_utc_ns == h.t_utc_ns && h2.fc_hz == h.fc_hz &&
	                            h2.stream_sample0 == h.stream_sample0 && h2.flags == h.flags &&
	                            h2.gain_tenths_db == 280 && h2.payload_bytes == 5u * 120000u * 2u,
	      NULL);
	for (i = 0; i < 5; i++)
		check("channel record survives",
		      ch2[i].delay == ch[i].delay && ch2[i].frac_delay == ch[i].frac_delay &&
		          ch2[i].w_im == ch[i].w_im && ch2[i].sample_count == ch[i].sample_count,
		      NULL);
	buf[30] ^= 1;
	check("a flipped bit fails the CRC", atkdaq_hdr_unpack(buf, n, &h2, ch2, 5) == -4, NULL);
	buf[30] ^= 1;
	buf[0] = 'X';
	check("bad magic is refused", atkdaq_hdr_unpack(buf, n, &h2, ch2, 5) == -2, NULL);
	buf[0] = 'A';
	buf[4] = 2;
	check("a damaged version field is damage, not a new format", atkdaq_hdr_unpack(buf, n, &h2, ch2, 5) == -4, NULL);
	{
		/* a genuinely newer header (CRC valid) is refused by version */
		uint32_t crc;
		buf[88] = buf[89] = buf[90] = buf[91] = 0;
		crc = atkdaq_crc32(0, buf, n);
		buf[88] = (uint8_t)crc; buf[89] = (uint8_t)(crc >> 8); buf[90] = (uint8_t)(crc >> 16); buf[91] = (uint8_t)(crc >> 24);
		check("a valid newer header is refused by version", atkdaq_hdr_unpack(buf, n, &h2, ch2, 5) == -3, NULL);
		buf[4] = 1;
		buf[88] = buf[89] = buf[90] = buf[91] = 0;
		crc = atkdaq_crc32(0, buf, n);
		buf[88] = (uint8_t)crc; buf[89] = (uint8_t)(crc >> 8); buf[90] = (uint8_t)(crc >> 16); buf[91] = (uint8_t)(crc >> 24);
	}
	check("a short buffer asks for more", atkdaq_hdr_unpack(buf, 100, &h2, ch2, 5) == -1, NULL);
}

static void test_convert(void)
{
	uint8_t raw[8] = {128, 127, 0, 200, 255, 128, 10, 20};
	int8_t out[8];
	uint32_t clip = atkdaq_u8_to_ci8(out, raw, 8);
	check("u8 -> int8 is XOR 0x80", out[0] == 0 && out[1] == -1 && out[2] == -128 && out[4] == 127, NULL);
	check("a sample with I or Q at the rail counts once", clip == 2, NULL);
}

static void test_ring(void)
{
	atkdaq_ring r;
	uint8_t src[10000], dst[10000];
	int i, rc;
	for (i = 0; i < 10000; i++)
		src[i] = (uint8_t)(i * 7);
	check("ring refuses a non-power-of-two", atkdaq_ring_init(&r, 5000) != 0, NULL);
	atkdaq_ring_init(&r, 8192);
	atkdaq_ring_write(&r, src, 6000);
	check("not yet written is -1", atkdaq_ring_read(&r, 5000, dst, 2000) == -1, NULL);
	rc = atkdaq_ring_read(&r, 1000, dst, 3000);
	check("a written range reads back", rc == 0 && memcmp(dst, src + 1000, 3000) == 0, NULL);
	atkdaq_ring_write(&r, src + 6000, 4000);      /* wraps: 10000 written into 8192 */
	rc = atkdaq_ring_read(&r, 5000, dst, 5000);
	check("a wrapped range reads back", rc == 0 && memcmp(dst, src + 5000, 5000) == 0, NULL);
	check("overwritten data is -2, never stale bytes", atkdaq_ring_read(&r, 1000, dst, 100) == -2, NULL);
	check("oldest readable position", atkdaq_ring_oldest(&r) == 10000 - 8192, NULL);
	check("a range longer than the ring is refused", atkdaq_ring_read(&r, 0, dst, 9000) == -3, NULL);
	{
		int8_t c8[100];
		uint32_t clip = 0;
		check("ci8 refuses an odd position", atkdaq_ring_read_ci8(&r, 5001, c8, 100, &clip) == -3, NULL);
		rc = atkdaq_ring_read_ci8(&r, 5000, c8, 100, &clip);
		check("ci8 read converts", rc == 0 && c8[0] == (int8_t)(src[5000] ^ 0x80), NULL);
	}
	atkdaq_ring_set_floor(&r, 10000 - 1000);
	check("free space follows the consumer's floor", atkdaq_ring_free_space(&r) == 8192 - 1000, NULL);
	atkdaq_ring_free(&r);
}

static void test_clock_and_drops(void)
{
	/* five channels, 131072-sample transfers at 2.4 MSPS, arrival latency
	 * 0.5 ms + uniform jitter up to 0.4 ms; channel starts offset; then
	 * channel 2 loses 5000 samples at transfer 60, and later every channel
	 * loses 3000 together (a common-mode stall) */
	atkdaq_sclock clk[5];
	const atkdaq_sclock *pc[5];
	atkdaq_drops dr;
	double fs = 2400000.0, P = 1e9 / fs;
	long start[5] = {0, 130, -40, 900, 12};
	long lost[5] = {0, 0, 0, 0, 0};
	uint64_t n[5] = {0, 0, 0, 0, 0};
	int T = 131072, k, i, flagged_at = -1;
	uint32_t mask = 0;
	int64_t t0 = 1000000000LL;
	char d[200];
	for (i = 0; i < 5; i++) {
		atkdaq_sclock_init(&clk[i], fs);
		pc[i] = &clk[i];
	}
	atkdaq_drops_init(&dr, 5, fs, 8, 1e6, 2);
	for (k = 0; k < 120; k++) {
		if (k == 60)
			lost[2] += 5000;
		if (k == 90)
			for (i = 0; i < 5; i++)
				lost[i] += 3000;
		for (i = 0; i < 5; i++) {
			double tend;
			n[i] += (uint64_t)T;
			tend = (double)t0 + ((double)n[i] + start[i] + lost[i]) * P + 5e5 + urand() * 4e5;
			atkdaq_sclock_add(&clk[i], n[i], (int64_t)tend);
		}
		if (k == 20)
			atkdaq_drops_rebase(&dr, pc);
		if (k > 20) {
			uint32_t m = atkdaq_drops_update(&dr, pc);
			if (m && flagged_at < 0) {
				flagged_at = k;
				mask = m;
			}
		}
		if (k == 85) {
			/* the clock: sample n's time within the jitter of the truth,
			 * checked away from any step (a loss moves the lower envelope
			 * only once the pre-loss transfers leave its window) */
			int64_t t = atkdaq_sclock_time(&clk[0], n[0] - 1000);
			double truth = (double)t0 + ((double)(n[0] - 1000) + start[0] + lost[0]) * P;
			snprintf(d, sizeof(d), "err=%.0f ns", (double)t - truth);
			check("the clock places a sample within 0.6 ms (min latency + one jitter floor)",
			      (double)t - truth > 0 && (double)t - truth < 6e5, d);
		}
	}
	snprintf(d, sizeof(d), "flagged_at=%d mask=%u est=%lld", flagged_at, mask, (long long)dr.est_samples[2]);
	check("a 5000-sample loss on channel 2 is flagged, on channel 2 only", mask == (1u << 2), d);
	check("...within a second of stream", flagged_at >= 60 && flagged_at <= 60 + 20, d);
	check("...and sized to within 1000 samples", llabs(dr.est_samples[2] - 5000) < 1000, d);
	snprintf(d, sizeof(d), "common=%lld", (long long)dr.common_samples);
	check("a loss every channel shares is common mode, not misalignment", llabs(dr.common_samples - 3000) < 1000, d);
}

/* Five channels of a common noise source through H_i(f) = g e^{jφ} e^{-j2πfτ/fs},
 * with independent receiver noise, integer offsets `shift` (channel i starts
 * shift[i] samples later in the common signal), as float I,Q. */
static void make_array(float **out, int n_ch, int n, const int *shift, const double *g_db,
                       const double *ph_deg, const double *tau, double snr_db)
{
	int B = 1;
	int i, k, extra = 0;
	double *common, *x;
	cfft_plan plan;
	for (i = 0; i < n_ch; i++)
		if (abs(shift[i]) > extra)
			extra = abs(shift[i]);
	while (B < n + 2 * extra + 64)
		B <<= 1;
	common = (double *)malloc(sizeof(double) * 2 * B);
	x = (double *)malloc(sizeof(double) * 2 * B);
	plan = make_cfft_plan((size_t)B);
	cnoise(common, B, (double)B * 0.05);
	for (i = 0; i < n_ch; i++) {
		double g = pow(10.0, g_db[i] / 20.0), ph = ph_deg[i] * M_PI / 180.0;
		double nvar = 0.05 * pow(10.0, -snr_db / 10.0);
		for (k = 0; k < B; k++) {
			int m = k < B / 2 ? k : k - B;
			double ang = ph - 2 * M_PI * m * tau[i] / B;
			double hr = g * cos(ang), hi = g * sin(ang);
			x[2 * k] = common[2 * k] * hr - common[2 * k + 1] * hi;
			x[2 * k + 1] = common[2 * k] * hi + common[2 * k + 1] * hr;
		}
		cfft_backward(plan, x, 1.0 / B);
		for (k = 0; k < n; k++) {
			double nr, ni, r = sqrt(-nvar * log(urand())), t = 2 * M_PI * urand();
			int src = k + extra - shift[i];      /* x_i[j + shift_i] = x_0[j] */
			nr = r * cos(t);
			ni = r * sin(t);
			out[i][2 * k] = (float)(x[2 * src] + nr);
			out[i][2 * k + 1] = (float)(x[2 * src + 1] + ni);
		}
	}
	destroy_cfft_plan(plan);
	free(common);
	free(x);
}

static void test_sync_and_cal(void)
{
	enum { N = 65536 };
	float *iq[5];
	const float *ciq[5];
	int shift[5] = {0, 17, -5, 120, 3};
	double gdb[5] = {0, -1.0, 0.5, -0.4, 1.2};
	double ph[5] = {0, 40, -100, 170, 20};
	double tau[5] = {0, 0.21, -0.33, 0.12, 0.44};
	double zeros[5] = {0, 0, 0, 0, 0};
	int zsh[5] = {0, 0, 0, 0, 0};
	atkdaq_sync_result s;
	atkdaq_cal_result c;
	int i, rc, ok;
	char d[256];
	for (i = 0; i < 5; i++) {
		iq[i] = (float *)malloc(sizeof(float) * 2 * N);
		ciq[i] = iq[i];
	}
	/* sync: channel i's sample j+shift[i] is channel 0's sample j */
	make_array(iq, 5, N, shift, gdb, ph, tau, 20.0);
	rc = atkdaq_sync_estimate(ciq, 5, N, 4096, &s);
	ok = rc == 0;
	for (i = 0; i < 5; i++)
		ok = ok && s.lag[i] == shift[i];
	snprintf(d, sizeof(d), "lags %d %d %d %d %d ptn %.1f dB", s.lag[0], s.lag[1], s.lag[2], s.lag[3], s.lag[4],
	         s.worst_ptn_db);
	check("sync finds every planted integer delay exactly", ok, d);
	check("sync peak-to-next is large on the noise source", s.worst_ptn_db > 20.0f, d);

	/* cal on aligned channels with planted gain, phase and fractional delay */
	make_array(iq, 5, N, zsh, gdb, ph, tau, 20.0);
	rc = atkdaq_cal_estimate(ciq, 5, N, 2400000.0, &c);
	ok = rc == 0;
	for (i = 1; i < 5; i++) {
		/* expected w = g0 e^{jφ0} / (g_i e^{jφ_i}) */
		double wdb = 20.0 * log10(hypot(c.w_re[i], c.w_im[i]));
		double wph = atan2(c.w_im[i], c.w_re[i]) * 180.0 / M_PI;
		double eph = fmod(-(ph[i] - ph[0]) + 540.0, 360.0) - 180.0;
		double dph = fmod(wph - eph + 540.0, 360.0) - 180.0;
		snprintf(d, sizeof(d), "ch%d: w %.3f dB (want %.3f) %.3f deg (want %.3f), frac %.4f (want %.4f), resid %.3f",
		         i, wdb, -(gdb[i] - gdb[0]), wph, eph, c.frac[i], tau[i] - tau[0], c.resid_deg[i]);
		check("cal: gain to 0.1 dB", fabs(wdb + (gdb[i] - gdb[0])) < 0.1, d);
		check("cal: phase to 0.5 deg", fabs(dph) < 0.5, d);
		check("cal: fractional delay to 0.01 sample", fabs(c.frac[i] - (tau[i] - tau[0])) < 0.01, d);
	}
	check("cal: residual under 2 deg", ok && c.spread_deg < 2.0f, NULL);

	/* no noise source: independent noise on every channel must be REFUSED */
	{
		int k;
		for (i = 0; i < 5; i++)
			for (k = 0; k < 2 * N; k++)
				iq[i][k] = (float)(urand() - 0.5);
	}
	rc = atkdaq_cal_estimate(ciq, 5, N, 2400000.0, &c);
	snprintf(d, sizeof(d), "rc=%d coherence %.3f", rc, c.coherence[1]);
	check("cal refuses when the noise source is not seen", rc == -3, d);
	(void)zeros;
	for (i = 0; i < 5; i++)
		free(iq[i]);
}

static void test_sched(void)
{
	atkdaq_config cfg;
	sched s;
	long long S = 1000000000LL;
	int i;
	atkdaq_config_defaults(&cfg);
	sched_init(&s, &cfg);
	sched_on_start(&s, 0, 1);
	check("start: sync + cal", sched_next(&s, 0) == PROC_FULL, NULL);
	sched_done(&s, PROC_FULL, 1 * S, 1, 1);
	check("nothing due right after", sched_next(&s, 2 * S) == PROC_NONE, NULL);
	/* retunes: full sync until three have kept the delays */
	for (i = 0; i < 3; i++) {
		sched_on_retune(&s, 3 * S);
		check("retune while the hypothesis is unproven: full sync", sched_next(&s, 3 * S) == PROC_FULL, NULL);
		sched_done(&s, PROC_FULL, 4 * S, 1, 0);
	}
	sched_on_retune(&s, 5 * S);
	check("retune once proven three times: cal only", sched_next(&s, 5 * S) == PROC_CAL, NULL);
	sched_done(&s, PROC_CAL, 6 * S, 1, 0);
	/* a periodic cal that finds the delays moved: sync now, hypothesis kept */
	{
		sched t = s;
		sched_request(&t, PROC_CAL);
		(void)sched_next(&t, 6 * S);
		sched_on_delays_moved(&t);
		check("a cal that finds the delays moved: sync now, no back-off", sched_next(&t, 6 * S) == PROC_FULL, NULL);
		check("...and a periodic cal does not refute the retune hypothesis", t.hyp >= 3, NULL);
		/* the same thing on a cal-only retune does refute it */
		t = s;
		sched_on_retune(&t, 6 * S);
		check("(cal-only retune)", sched_next(&t, 6 * S) == PROC_CAL, NULL);
		sched_on_delays_moved(&t);
		check("a cal-only retune whose cal finds the delays moved: sync now",
		      sched_next(&t, 6 * S) == PROC_FULL, NULL);
		check("...and the hypothesis is refuted for the session", t.hyp < 0, NULL);
		sched_done(&t, PROC_FULL, 7 * S, 1, 1);
		sched_on_retune(&t, 8 * S);
		check("...so the next retune gets a full sync", sched_next(&t, 8 * S) == PROC_FULL, NULL);
	}
	/* gain change: cal */
	sched_on_gain(&s, 7 * S);
	check("gain change: cal", sched_next(&s, 7 * S) == PROC_CAL, NULL);
	sched_done(&s, PROC_CAL, 8 * S, 1, 0);
	/* misaligned, static: waits for quiet up to 5 s */
	sched_on_misaligned(&s, 10 * S);
	check("static misalignment waits for a quiet moment", sched_next(&s, 11 * S) == PROC_NONE, NULL);
	sched_on_quiet(&s);
	check("...and syncs when told quiet", sched_next(&s, 11 * S) == PROC_FULL, NULL);
	sched_done(&s, PROC_FULL, 12 * S, 1, 0);
	sched_on_misaligned(&s, 20 * S);
	check("...or after quiet_wait_s anyway", sched_next(&s, 26 * S) == PROC_FULL, NULL);
	sched_done(&s, PROC_FULL, 27 * S, 1, 0);
	/* mobile: immediately */
	sched_set_mode(&s, ATKDAQ_MODE_MOBILE);
	sched_on_misaligned(&s, 30 * S);
	check("mobile misalignment syncs immediately", sched_next(&s, 30 * S) == PROC_FULL, NULL);
	/* failure backs off, a request overrides */
	sched_done(&s, PROC_FULL, 31 * S, 0, 0);
	check("a failure backs off", sched_next(&s, 32 * S) == PROC_NONE, NULL);
	check("...and retries", sched_next(&s, 34 * S) == PROC_FULL, NULL);
	sched_done(&s, PROC_FULL, 35 * S, 1, 0);
	/* a retune that moves a delay refutes the hypothesis */
	sched_set_mode(&s, ATKDAQ_MODE_STATIC);
	s.hyp = 0;
	sched_on_retune(&s, 40 * S);
	sched_next(&s, 40 * S);
	sched_done(&s, PROC_FULL, 41 * S, 1, 1);
	check("a retune that moved the delays refutes the hypothesis", s.hyp < 0, NULL);
	sched_on_retune(&s, 42 * S);
	check("...so every later retune gets a full sync", sched_next(&s, 42 * S) == PROC_FULL, NULL);
	sched_done(&s, PROC_FULL, 43 * S, 1, 0);
	/* periodic check in mobile mode */
	sched_set_mode(&s, ATKDAQ_MODE_MOBILE);
	check("mobile: periodic check after sync_check_mobile_s", sched_next(&s, 43 * S + 61 * S) == PROC_CHECK, NULL);
	/* the operator holding the noise source stops the policy */
	sched_hold_noise(&s, 1);
	check("a held noise source suspends the policy", sched_next(&s, 200 * S) == PROC_NONE, NULL);
}

static void test_control_and_config(void)
{
	command c;
	char err[256];
	atkdaq_config cfg;
	check("tune 446e6", control_parse("tune 446e6", &c, err, 256) == 0 && c.kind == CMD_TUNE && c.ival == 446000000, NULL);
	check("gain 280", control_parse("gain 280", &c, err, 256) == 0 && c.kind == CMD_GAIN && c.ival == 280, NULL);
	check("noise auto", control_parse("  noise auto ", &c, err, 256) == 0 && c.ival == 2, NULL);
	check("interval cal 30", control_parse("interval cal 30", &c, err, 256) == 0 && c.what == 1 && c.ival == 30, NULL);
	check("a blank line is nothing", control_parse("   ", &c, err, 256) == 1, NULL);
	check("nonsense is refused with a sentence", control_parse("tune fast", &c, err, 256) == -1 && strlen(err) > 10, NULL);
	atkdaq_config_defaults(&cfg);
	check("serial list", atkdaq_config_set(&cfg, "serial", "1000, 1001,1002,1003,1004", err, 256) == 0 &&
	                         !strcmp(cfg.serial[3], "1003"), NULL);
	check("an unknown key is refused", atkdaq_config_set(&cfg, "gian", "280", err, 256) == -1, NULL);
	check("fc in exponent form", atkdaq_config_set(&cfg, "fc_hz", "433.92e6", err, 256) == 0 && cfg.fc_hz == 433920000ULL, NULL);
	check("emitter needs one phase per channel", atkdaq_config_set(&cfg, "synth.emitter", "1000,-20,0,0", err, 256) == -2, NULL);
	check("emitter parses", atkdaq_config_set(&cfg, "synth.emitter", "25000,-20,0,72,144,216,288", err, 256) == 0 &&
	                            cfg.synth_emitter[0].active && cfg.synth_emitter[0].steer_deg[4] == 288.0, NULL);
	check("defaults validate", atkdaq_config_validate(&cfg, err, 256) == 0, err);
	cfg.fs_hz = 2048001;
	check("a frame that is not a whole number of samples is refused", atkdaq_config_validate(&cfg, err, 256) != 0, err);
}

int main(void)
{
	printf("atkdaq smoke test (core %s, ABI %d)\n", atkdaq_version(), atkdaq_core_abi());
	test_crc_and_header();
	test_convert();
	test_ring();
	test_clock_and_drops();
	test_sync_and_cal();
	test_sched();
	test_control_and_config();
	printf("%d passed, %d failed\n", g_pass, g_fail);
	return g_fail ? 1 : 0;
}
