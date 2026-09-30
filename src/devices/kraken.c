/*
 * kraken.c — the KrakenSDR: five R820T + RTL2832U chains on one 28.8 MHz
 * clock with a switchable noise source, driven through the krakenrf fork of
 * librtlsdr (vendor/librtlsdr, pinned — see PINNED.txt).
 *
 * WHAT MUST BE TRUE FOR THE FIVE CHANNELS TO BE COHERENT, and where each is
 * done below:
 *
 *   1. Every chain on the SAME frequency with the SAME manual gain, AGC off
 *      (tuner and RTL2832U both) — configure_one().
 *   2. PLL DITHERING OFF, and BEFORE the first frequency is set: the R820T's
 *      fractional-N PLL dithers its divider by default, which smears each
 *      tuner's phase independently and makes the array incoherent no matter
 *      what is calibrated afterwards. rtlsdr_set_dithering() exists only in
 *      the fork; its header says it must run before freq_set, and Heimdall
 *      (KrakenSDR's own firmware) calls it first. A failure refuses the run
 *      — stock librtlsdr, or a non-R820T tuner, can never be coherent.
 *   3. The channels opened by EEPROM SERIAL, in the configured order, and
 *      refused if any is missing — serial order IS channel order.
 *   4. The noise source is GPIO 0 of channel 0's RTL2832U, and channel m's
 *      bias tee is GPIO m+1 of the same chip (Heimdall's usage). The generic
 *      rtlsdr_set_bias_tee() is NEVER called here: on channel 0 it is the
 *      noise source.
 *
 * THREADS. One reader thread per device, each inside rtlsdr_read_async
 * (librtlsdr gives every device its own libusb context, so each thread
 * services its own events — Heimdall's arrangement). The callback stamps the
 * monotonic clock and hands the bytes to atkdaq; it never calls librtlsdr.
 * Retunes and gain changes are made from the assembler thread while the
 * streams run — also Heimdall's arrangement; the fork serialises its I2C
 * access with a recursive mutex.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../plat.h"
#include "atkdaq_core.h"
#include "device.h"

#ifdef ATKDAQ_HAVE_RTLSDR
#include "rtl-sdr.h"

struct atkdaq_dev;

typedef struct kctx {
	struct atkdaq_dev *d;
	int ch;
} kctx;

struct atkdaq_dev {
	atkdaq_config cfg;
	int n_ch;
	rtlsdr_dev_t *dev[ATKDAQ_MAX_CHANNELS];
	uint32_t index[ATKDAQ_MAX_CHANNELS];
	plat_thread th[ATKDAQ_MAX_CHANNELS];
	int th_started[ATKDAQ_MAX_CHANNELS];
	kctx ctx[ATKDAQ_MAX_CHANNELS];
	atkdaq_on_data on_data;
	atkdaq_on_fault on_fault;
	void *user;
	volatile int running;
	/* start barrier: every reader resets its buffer and starts together */
	plat_mutex bar_lock;
	plat_cond bar_cond;
	int bar_count;
	char desc[1024];
	char product[520];
};

static const char *tuner_name(enum rtlsdr_tuner t)
{
	switch (t) {
	case RTLSDR_TUNER_E4000: return "E4000";
	case RTLSDR_TUNER_FC0012: return "FC0012";
	case RTLSDR_TUNER_FC0013: return "FC0013";
	case RTLSDR_TUNER_FC2580: return "FC2580";
	case RTLSDR_TUNER_R820T: return "R820T";
	case RTLSDR_TUNER_R828D: return "R828D";
	default: return "unknown";
	}
}

static void present_serials(char *buf, int len)
{
	uint32_t n = rtlsdr_get_device_count(), i;
	int off = 0;
	buf[0] = 0;
	for (i = 0; i < n && off < len - 24; i++) {
		char m[256], p[256], s[256];
		m[0] = p[0] = s[0] = 0;
		if (rtlsdr_get_device_usb_strings(i, m, p, s) == 0)
			off += snprintf(buf + off, len - off, "%s%s", i ? ", " : "", s[0] ? s : "(blank)");
		else
			off += snprintf(buf + off, len - off, "%s(unreadable)", i ? ", " : "");
	}
	if (n == 0)
		snprintf(buf, len, "none");
}

static int configure_one(atkdaq_dev *d, int i, char *err, int errlen)
{
	rtlsdr_dev_t *dev = d->dev[i];
	const char *ser = d->cfg.serial[i];
	enum rtlsdr_tuner t = rtlsdr_get_tuner_type(dev);
	uint32_t got;
	/* Only the R820T is accepted: a KrakenSDR's R820T2 enumerates as
	 * RTLSDR_TUNER_R820T, and the fork's rtlsdr_set_dithering ONLY handles the
	 * R820T (it returns 1 for anything else). Accepting R828D here would let it
	 * reach the dithering call below and fail with the wrong message ("could
	 * not be turned off") when the real reason is the fork does not control
	 * dithering on an R828D. */
	if (t != RTLSDR_TUNER_R820T) {
		snprintf(err, errlen,
		         "channel %d (serial %s) has a %s tuner; a KrakenSDR has R820T tuners and only those "
		         "can have dithering turned off",
		         i, ser, tuner_name(t));
		return -1;
	}
	/* 2. dithering off, BEFORE the first frequency */
	if (rtlsdr_set_dithering(dev, 0) != 0) {
		snprintf(err, errlen,
		         "channel %d (serial %s): PLL dithering could not be turned off. Without that the five "
		         "tuners are never phase-coherent - this needs the krakenrf librtlsdr fork (built in) "
		         "and an R820T tuner",
		         i, ser);
		return -1;
	}
	if (rtlsdr_set_tuner_gain_mode(dev, 1) != 0) {
		snprintf(err, errlen, "channel %d (serial %s): cannot select manual gain", i, ser);
		return -1;
	}
	if (rtlsdr_set_center_freq(dev, (uint32_t)d->cfg.fc_hz) != 0) {
		snprintf(err, errlen, "channel %d (serial %s): cannot tune to %llu Hz", i, ser,
		         (unsigned long long)d->cfg.fc_hz);
		return -1;
	}
	if (rtlsdr_set_tuner_gain(dev, d->cfg.gain_tenths) != 0) {
		snprintf(err, errlen, "channel %d (serial %s): cannot set gain %d", i, ser, d->cfg.gain_tenths);
		return -1;
	}
	if (rtlsdr_set_sample_rate(dev, d->cfg.fs_hz) != 0) {
		snprintf(err, errlen, "channel %d (serial %s): cannot set sample rate %u", i, ser, d->cfg.fs_hz);
		return -1;
	}
	got = rtlsdr_get_sample_rate(dev);
	if (got != d->cfg.fs_hz) {
		snprintf(err, errlen,
		         "channel %d (serial %s): asked for %u samples/s and the RTL2832U set %u - use a rate "
		         "it can make exactly (2400000 is exact)",
		         i, ser, d->cfg.fs_hz, got);
		return -1;
	}
	if (rtlsdr_set_agc_mode(dev, 0) != 0) {
		snprintf(err, errlen, "channel %d (serial %s): cannot turn the RTL2832U AGC off", i, ser);
		return -1;
	}
	return 0;
}

static void close_all(atkdaq_dev *d)
{
	int i;
	if (d->dev[0]) {
		/* leave the box as found: noise source and bias tees off */
		rtlsdr_set_bias_tee_gpio(d->dev[0], d->cfg.noise_gpio, 0);
		for (i = 0; i < d->n_ch; i++)
			rtlsdr_set_bias_tee_gpio(d->dev[0], i + 1, 0);
	}
	for (i = 0; i < d->n_ch; i++)
		if (d->dev[i]) {
			rtlsdr_close(d->dev[i]);
			d->dev[i] = NULL;
		}
}

static int kraken_open(atkdaq_dev **out, const atkdaq_config *cfg, char *err, int errlen)
{
	atkdaq_dev *d;
	uint32_t count;
	int i, j;
	char present[512];
	*out = NULL;
	count = rtlsdr_get_device_count();
	if (count == 0) {
		snprintf(err, errlen,
		         "no RTL-SDR devices found. Is the KrakenSDR connected and powered from its own "
		         "5 V >= 2.4 A supply, and has Zadig installed WinUSB on EACH of its five interfaces "
		         "(README, 'First run')?");
		return -1;
	}
	d = (atkdaq_dev *)calloc(1, sizeof(*d));
	if (!d) {
		snprintf(err, errlen, "out of memory");
		return -1;
	}
	d->cfg = *cfg;
	d->n_ch = cfg->n_channels;
	plat_mutex_init(&d->bar_lock);
	plat_cond_init(&d->bar_cond);

	/* 3. every serial present, each once */
	for (i = 0; i < d->n_ch; i++) {
		int idx = rtlsdr_get_index_by_serial(cfg->serial[i]);
		if (idx < 0) {
			present_serials(present, sizeof(present));
			snprintf(err, errlen,
			         "channel %d: serial %s not found. Serials present: %s. (A KrakenSDR ships as "
			         "1000-1004; `atkdaq enum` lists what is connected.)",
			         i, cfg->serial[i], present);
			goto fail;
		}
		for (j = 0; j < i; j++)
			if ((int)d->index[j] == idx) {
				snprintf(err, errlen, "serial %s is listed for channels %d and %d", cfg->serial[i], j, i);
				goto fail;
			}
		d->index[i] = (uint32_t)idx;
	}
	for (i = 0; i < d->n_ch; i++) {
		if (rtlsdr_open(&d->dev[i], d->index[i]) != 0) {
			d->dev[i] = NULL;
			snprintf(err, errlen,
			         "could not open channel %d (serial %s). Another program may have it (SDR#, "
			         "rtl_sdr, an earlier atkdaq still running), or its driver is not WinUSB.",
			         i, cfg->serial[i]);
			goto fail;
		}
	}
	for (i = 0; i < d->n_ch; i++)
		if (configure_one(d, i, err, errlen) != 0)
			goto fail;
	/* 4. noise source off, bias tees as configured, all through channel 0 */
	if (rtlsdr_set_bias_tee_gpio(d->dev[0], cfg->noise_gpio, 0) != 0) {
		snprintf(err, errlen, "cannot drive the noise-source GPIO %d on channel 0", cfg->noise_gpio);
		goto fail;
	}
	/* Per-channel bias tees on GPIO 1..n of channel 0's chip (Heimdall's
	 * mapping; UNVERIFIED until hardware, see plan §11.13 - note GPIO 4 is
	 * documented as tuner-reset on a stock RTL2832U, so confirm the Kraken
	 * board's wiring at bring-up). Only driven when the operator asked for the
	 * bias tees; left untouched otherwise rather than forcing lines low. */
	if (cfg->bias_tee)
		for (i = 0; i < d->n_ch; i++)
			rtlsdr_set_bias_tee_gpio(d->dev[0], i + 1, 1);
	{
		char m[256], p[256], s[256];
		m[0] = p[0] = s[0] = 0;
		rtlsdr_get_device_usb_strings(d->index[0], m, p, s);
		snprintf(d->product, sizeof(d->product), "%s %s", m, p);
		snprintf(d->desc, sizeof(d->desc),
		         "%s: %d x %s, serials %s..%s, %u samples/s, dithering off, AGC off, gain %.1f dB",
		         d->product[0] != ' ' ? d->product : "RTL2832U", d->n_ch,
		         tuner_name(rtlsdr_get_tuner_type(d->dev[0])), cfg->serial[0], cfg->serial[d->n_ch - 1],
		         cfg->fs_hz, rtlsdr_get_tuner_gain(d->dev[0]) / 10.0);
	}
	*out = d;
	return 0;
fail:
	close_all(d);
	plat_cond_destroy(&d->bar_cond);
	plat_mutex_destroy(&d->bar_lock);
	free(d);
	return -1;
}

static void kraken_close(atkdaq_dev *d)
{
	if (!d)
		return;
	close_all(d);
	plat_cond_destroy(&d->bar_cond);
	plat_mutex_destroy(&d->bar_lock);
	free(d);
}

static void kraken_stop(atkdaq_dev *d);   /* kraken_start unwinds through it on a failed launch */

static void rx_cb(unsigned char *buf, uint32_t len, void *ctx)
{
	kctx *k = (kctx *)ctx;
	atkdaq_dev *d = k->d;
	int64_t t = plat_mono_ns();
	if (!d->running)
		return;
	d->on_data(d->user, k->ch, buf, len, t);
}

static void reader(void *arg)
{
	kctx *k = (kctx *)arg;
	atkdaq_dev *d = k->d;
	int rc;
	/* barrier: start the five streams as close together as the OS allows.
	 * Not required — sync measures whatever offset results — but it keeps
	 * the offsets inside the coarse-alignment search from the first second. */
	plat_mutex_lock(&d->bar_lock);
	d->bar_count++;
	plat_cond_broadcast(&d->bar_cond);
	while (d->bar_count < d->n_ch && d->running)
		plat_cond_wait(&d->bar_cond, &d->bar_lock, 100);
	plat_mutex_unlock(&d->bar_lock);
	/* A stop, or a start that failed to launch every reader, clears running
	 * while threads sit at the barrier. Do NOT enter read_async then: an
	 * orphaned reader would stream forever while kraken_start's caller closes
	 * the device out from under it (use-after-free), and rtlsdr_cancel_async
	 * only arms once the read is RUNNING, so a reader that slips into it after
	 * stop already cancelled would never be joinable. */
	if (!d->running)
		return;
	rtlsdr_reset_buffer(d->dev[k->ch]);
	rc = rtlsdr_read_async(d->dev[k->ch], rx_cb, k, (uint32_t)d->cfg.transfer_count,
	                       (uint32_t)d->cfg.transfer_bytes);
	if (d->running) {
		char what[160];
		snprintf(what, sizeof(what), "channel %d (serial %s) stream ended (rc %d) - unplugged, or a USB error",
		         k->ch, d->cfg.serial[k->ch], rc);
		if (d->on_fault)
			d->on_fault(d->user, k->ch, what);
	}
}

static int kraken_start(atkdaq_dev *d, atkdaq_on_data on_data, atkdaq_on_fault on_fault, void *user)
{
	int i;
	d->on_data = on_data;
	d->on_fault = on_fault;
	d->user = user;
	d->running = 1;
	d->bar_count = 0;
	for (i = 0; i < d->n_ch; i++) {
		d->ctx[i].d = d;
		d->ctx[i].ch = i;
		if (plat_thread_start(&d->th[i], reader, &d->ctx[i]) != 0) {
			/* Tear the partially-started set down before returning: clear
			 * running (so the readers already past the barrier return before
			 * read_async), release the barrier, and join them. Otherwise the
			 * caller closes devices whose readers are still live. */
			kraken_stop(d);
			return -1;
		}
		d->th_started[i] = 1;
	}
	return 0;
}

static void kraken_stop(atkdaq_dev *d)
{
	int i, tries;
	d->running = 0;
	plat_mutex_lock(&d->bar_lock);
	plat_cond_broadcast(&d->bar_cond);
	plat_mutex_unlock(&d->bar_lock);
	/* rtlsdr_cancel_async only arms once the read is RUNNING and is a no-op
	 * before that. A reader that passed the barrier's running check but has
	 * not yet entered read_async would miss a single cancel and then loop for
	 * ever (the callback resubmits every transfer). So cancel repeatedly until
	 * it takes: cancel_async returns 0 when it armed, non-zero when there was
	 * nothing running to cancel. A device whose reader never started, or
	 * already ended, simply keeps returning non-zero and the join is instant. */
	for (i = 0; i < d->n_ch; i++) {
		if (!d->dev[i] || !d->th_started[i])
			continue;
		for (tries = 0; tries < 20; tries++) {
			if (rtlsdr_cancel_async(d->dev[i]) == 0)
				break;          /* armed: the reader will return from read_async */
			plat_sleep_ms(5);   /* ~100 ms covers reset_buffer + read_async entry */
		}
	}
	for (i = 0; i < d->n_ch; i++)
		if (d->th_started[i]) {
			plat_thread_join(d->th[i]);
			d->th_started[i] = 0;
		}
}

static int kraken_tune(atkdaq_dev *d, uint64_t fc_hz, uint64_t *n_eff)
{
	int i, rc = 0;
	if (fc_hz > 0xFFFFFFFFull)
		return -1;
	for (i = 0; i < d->n_ch; i++)
		if (rtlsdr_set_center_freq(d->dev[i], (uint32_t)fc_hz) != 0)
			rc = -1;
	if (n_eff)
		*n_eff = ATKDAQ_UNKNOWN_INDEX;
	return rc;
}

static int kraken_set_gain(atkdaq_dev *d, int tenths, int *applied, uint64_t *n_eff)
{
	int i, rc = 0;
	for (i = 0; i < d->n_ch; i++)
		if (rtlsdr_set_tuner_gain(d->dev[i], tenths) != 0)
			rc = -1;
	if (applied)
		*applied = rtlsdr_get_tuner_gain(d->dev[0]);
	if (n_eff)
		*n_eff = ATKDAQ_UNKNOWN_INDEX;
	return rc;
}

static int kraken_noise(atkdaq_dev *d, int on, uint64_t *n_eff)
{
	if (n_eff)
		*n_eff = ATKDAQ_UNKNOWN_INDEX;
	return rtlsdr_set_bias_tee_gpio(d->dev[0], d->cfg.noise_gpio, on ? 1 : 0) == 0 ? 0 : -1;
}

static int kraken_pll_locked(atkdaq_dev *d)
{
	int i, unknown = 0;
	for (i = 0; i < d->n_ch; i++) {
		int r = rtlsdr_is_tuner_PLL_locked(d->dev[i]);
		if (r == 1)
			return 0;
		if (r < 0)
			unknown = 1;
	}
	return unknown ? -1 : 1;
}

static int64_t kraken_now(atkdaq_dev *d)
{
	(void)d;
	return plat_mono_ns();
}

static void kraken_describe(atkdaq_dev *d, char *buf, int len)
{
	snprintf(buf, len, "%s", d->desc);
}

static int kraken_gains(atkdaq_dev *d, int *list, int max)
{
	int n = rtlsdr_get_tuner_gains(d->dev[0], NULL);
	int tmp[64];
	int i;
	if (n <= 0 || n > 64)
		return 0;
	n = rtlsdr_get_tuner_gains(d->dev[0], tmp);
	for (i = 0; i < n && i < max; i++)
		list[i] = tmp[i];
	return i;
}

static void kraken_set_rings(atkdaq_dev *d, struct atkdaq_ring *const *rings, int n)
{
	(void)d;
	(void)rings;
	(void)n;
}

int atkdaq_kraken_enumerate(void)
{
	uint32_t n = rtlsdr_get_device_count(), i;
	printf("RTL-SDR devices: %u\n", n);
	for (i = 0; i < n; i++) {
		char m[256], p[256], s[256];
		m[0] = p[0] = s[0] = 0;
		if (rtlsdr_get_device_usb_strings(i, m, p, s) == 0)
			printf("  index=%u serial=%s product=\"%s\" manufacturer=\"%s\"\n", i, s, p, m);
		else
			printf("  index=%u (strings unreadable - driver not WinUSB?)\n", i);
	}
	return (int)n;
}

#else /* no librtlsdr in this build */

struct atkdaq_dev {
	int unused;
};

static int kraken_open(atkdaq_dev **out, const atkdaq_config *cfg, char *err, int errlen)
{
	(void)cfg;
	*out = NULL;
	snprintf(err, errlen, "this atkdaq was built without librtlsdr, so it has no KrakenSDR support "
	                      "(only --device synth). build.bat builds it with the fork.");
	return -1;
}
static void kraken_close(atkdaq_dev *d) { (void)d; }
static int kraken_start(atkdaq_dev *d, atkdaq_on_data a, atkdaq_on_fault b, void *u)
{
	(void)d; (void)a; (void)b; (void)u;
	return -1;
}
static void kraken_stop(atkdaq_dev *d) { (void)d; }
static int kraken_tune(atkdaq_dev *d, uint64_t f, uint64_t *n) { (void)d; (void)f; (void)n; return -1; }
static int kraken_set_gain(atkdaq_dev *d, int t, int *a, uint64_t *n)
{
	(void)d; (void)t; (void)a; (void)n;
	return -1;
}
static int kraken_noise(atkdaq_dev *d, int on, uint64_t *n) { (void)d; (void)on; (void)n; return -1; }
static int kraken_pll_locked(atkdaq_dev *d) { (void)d; return -1; }
static int64_t kraken_now(atkdaq_dev *d) { (void)d; return plat_mono_ns(); }
static void kraken_describe(atkdaq_dev *d, char *buf, int len) { (void)d; snprintf(buf, len, "none"); }
static int kraken_gains(atkdaq_dev *d, int *l, int m) { (void)d; (void)l; (void)m; return 0; }
static void kraken_set_rings(atkdaq_dev *d, struct atkdaq_ring *const *r, int n) { (void)d; (void)r; (void)n; }
int atkdaq_kraken_enumerate(void)
{
	printf("this atkdaq was built without librtlsdr\n");
	return -1;
}
#endif

const atkdaq_dev_ops atkdaq_kraken_ops = {
	"kraken", ATKDAQ_DEV_KRAKEN,
	kraken_open, kraken_close, kraken_start, kraken_stop,
	kraken_tune, kraken_set_gain, kraken_noise, kraken_pll_locked,
	kraken_now, kraken_describe, kraken_gains, kraken_set_rings,
};
