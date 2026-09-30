/*
 * play.c — `atkdaq play FILE.atkq`: a recording replayed as if live.
 *
 * Frames are written to stdout verbatim (a recording IS a stream), paced by
 * their own sample counters: frame k goes out when (stream_sample0 - first)
 * / fs seconds of wall clock have passed since the first (divided by
 * --speed), so a consumer sees the cadence the DAQ produced — and a gap the
 * DAQ recorded (dropped frames, an overrun) is a gap in time here too.
 * --fast removes the pacing. --loop starts again at the end; the consumer
 * can tell because stream_sample0 goes backwards.
 *
 * A damaged file is survived, not trusted: bytes that do not start a header
 * whose CRC checks are skipped until one does, and the skip is reported on
 * stderr. Nothing is written that did not validate.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "atkdaq_core.h"
#include "log.h"
#include "plat.h"

int atkdaq_play_main(const char *path, int loop, int fast, double speed);

typedef struct rbuf {
	FILE *f;
	uint8_t *p;
	size_t cap, pos, end;
	int eof;
} rbuf;

/* make at least n bytes available from pos; 0 ok, -1 end of file first */
static int need(rbuf *b, size_t n)
{
	while (b->end - b->pos < n) {
		size_t got;
		if (b->eof)
			return -1;
		if (b->pos > 0) {
			memmove(b->p, b->p + b->pos, b->end - b->pos);
			b->end -= b->pos;
			b->pos = 0;
		}
		if (b->cap - b->end < (1u << 20) || b->cap < n) {
			size_t nc = b->cap ? b->cap * 2 : (4u << 20);
			uint8_t *np;
			while (nc < n + (1u << 20))
				nc *= 2;
			np = (uint8_t *)realloc(b->p, nc);
			if (!np)
				return -1;
			b->p = np;
			b->cap = nc;
		}
		got = fread(b->p + b->end, 1, b->cap - b->end, b->f);
		if (got == 0)
			b->eof = 1;
		b->end += got;
	}
	return 0;
}

int atkdaq_play_main(const char *path, int loop, int fast, double speed)
{
	rbuf b;
	plat_out *out;
	uint64_t frames = 0, skipped = 0;
	int rc = 0, stop = 0;
	memset(&b, 0, sizeof(b));
	if (speed <= 0)
		speed = 1.0;
	b.f = fopen(path, "rb");
	if (!b.f) {
		log_event("error", "msg=\"cannot open %s\"", path);
		return 2;
	}
	plat_binary_stdio();
	out = plat_out_stdout();
	log_event("play", "file=\"%s\" loop=%d fast=%d speed=%.3f", path, loop, fast, speed);
	while (!stop) {
		int64_t t0 = plat_mono_ns();
		uint64_t first_s0 = 0;
		int have_first = 0;
		for (;;) {
			atkdaq_hdr h;
			atkdaq_ch ch[ATKDAQ_MAX_CHANNELS];
			int hn;
			size_t total;
			uint16_t hb;
			if (need(&b, ATKDAQ_FIXED_BYTES) != 0)
				break;
			if (memcmp(b.p + b.pos, ATKDAQ_MAGIC, 4) != 0) {
				b.pos++;
				skipped++;
				continue;
			}
			hb = (uint16_t)(b.p[b.pos + 6] | (b.p[b.pos + 7] << 8));
			if (hb < ATKDAQ_FIXED_BYTES || hb > ATKDAQ_HDR_BYTES(ATKDAQ_MAX_CHANNELS) ||
			    need(&b, hb) != 0) {
				b.pos++;
				skipped++;
				continue;
			}
			hn = atkdaq_hdr_unpack(b.p + b.pos, hb, &h, ch, ATKDAQ_MAX_CHANNELS);
			if (hn <= 0) {
				b.pos++;
				skipped++;
				continue;
			}
			total = (size_t)h.hdr_bytes + h.payload_bytes;
			if (need(&b, total) != 0)
				break;           /* a truncated last frame is not replayed */
			if (skipped) {
				log_event("resync", "skipped_bytes=%llu", (unsigned long long)skipped);
				skipped = 0;
			}
			if (!have_first) {
				first_s0 = h.stream_sample0;
				have_first = 1;
			}
			if (!fast && h.fs_hz > 0 && h.stream_sample0 >= first_s0) {
				double secs = (double)(h.stream_sample0 - first_s0) / (double)h.fs_hz / speed;
				plat_sleep_until_ns(t0 + (int64_t)(secs * 1e9));
			}
			if (plat_out_write(out, b.p + b.pos, total) != 0) {
				stop = 1;        /* the consumer has gone */
				break;
			}
			b.pos += total;
			frames++;
		}
		if (stop || !loop || frames == 0)
			break;
		if (fseek(b.f, 0, SEEK_SET) != 0)
			break;
		b.pos = b.end = 0;
		b.eof = 0;
	}
	if (skipped)
		log_event("resync", "skipped_bytes=%llu at_end=1", (unsigned long long)skipped);
	log_event("exit", "frames=%llu code=%d", (unsigned long long)frames, rc);
	free(b.p);
	fclose(b.f);
	return rc;
}
