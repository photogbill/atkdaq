/*
 * frame.c — the header, its CRC, and the payload conversion.
 *
 * The header is serialised field by field in little-endian order rather than
 * memcpy'd from the struct, so the bytes on the wire are the same whatever
 * the compiler did with the struct; the static asserts below still check the
 * struct matches the documented layout, because tests and readers in other
 * languages are written against the offsets in atkdaq_frame.h.
 */
#include <stddef.h>
#include <string.h>

#include "atkdaq_core.h"
#include "version.h"

#define STATIC_ASSERT(cond, name) typedef char static_assert_##name[(cond) ? 1 : -1]

STATIC_ASSERT(sizeof(atkdaq_hdr) == ATKDAQ_FIXED_BYTES, fixed_part_is_96_bytes);
STATIC_ASSERT(sizeof(atkdaq_ch) == ATKDAQ_CH_STRIDE, channel_record_is_32_bytes);
STATIC_ASSERT(offsetof(atkdaq_hdr, t_utc_ns) == 24, t_utc_at_24);
STATIC_ASSERT(offsetof(atkdaq_hdr, stream_sample0) == 72, stream_sample0_at_72);
STATIC_ASSERT(offsetof(atkdaq_hdr, hdr_crc32) == 88, crc_at_88);
STATIC_ASSERT(offsetof(atkdaq_ch, sample_count) == 24, sample_count_at_24);

int atkdaq_core_abi(void) { return ATKDAQ_CORE_ABI; }
const char *atkdaq_version(void) { return ATKDAQ_VERSION_STRING; }

/* ------------------------------------------------------------------------ */
/* CRC-32, IEEE 802.3 (reflected, poly 0xEDB88320) — what zlib.crc32 computes */
/* ------------------------------------------------------------------------ */

static uint32_t crc_table[256];
static int crc_ready;

static void crc_init(void)
{
	uint32_t i, j, c;
	for (i = 0; i < 256; i++) {
		c = i;
		for (j = 0; j < 8; j++)
			c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
		crc_table[i] = c;
	}
	crc_ready = 1;
}

uint32_t atkdaq_crc32(uint32_t crc, const void *data, size_t n)
{
	const uint8_t *p = (const uint8_t *)data;
	size_t i;
	if (!crc_ready)
		crc_init();
	crc = ~crc;
	for (i = 0; i < n; i++)
		crc = crc_table[(crc ^ p[i]) & 0xFFu] ^ (crc >> 8);
	return ~crc;
}

/* ------------------------------------------------------------------------ */
/* little-endian field writers/readers                                       */
/* ------------------------------------------------------------------------ */

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static void put64(uint8_t *p, uint64_t v) { put32(p, (uint32_t)v); put32(p + 4, (uint32_t)(v >> 32)); }
static void putf(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); put32(p, u); }

static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint64_t get64(const uint8_t *p) { return (uint64_t)get32(p) | ((uint64_t)get32(p + 4) << 32); }
static float getf(const uint8_t *p) { uint32_t u = get32(p); float f; memcpy(&f, &u, 4); return f; }

size_t atkdaq_hdr_pack(uint8_t *out, size_t cap, const atkdaq_hdr *h, const atkdaq_ch *ch, int n)
{
	size_t total;
	int i;
	uint32_t crc;
	if (n < 1 || n > ATKDAQ_MAX_CHANNELS)
		return 0;
	total = (size_t)ATKDAQ_HDR_BYTES(n);
	if (cap < total)
		return 0;
	memset(out, 0, total);
	memcpy(out + 0, ATKDAQ_MAGIC, 4);
	put16(out + 4, ATKDAQ_VERSION);
	put16(out + 6, (uint16_t)total);
	put16(out + 8, ATKDAQ_FIXED_BYTES);
	put16(out + 10, ATKDAQ_CH_STRIDE);
	out[12] = (uint8_t)n;
	out[13] = h->fmt;
	out[14] = h->cal_state;
	out[15] = h->device_kind;
	put32(out + 16, h->flags);
	put32(out + 20, h->seq);
	put64(out + 24, (uint64_t)h->t_utc_ns);
	put64(out + 32, h->fc_hz);
	put32(out + 40, h->fs_hz);
	put32(out + 44, h->samples_per_ch);
	put32(out + 48, h->segment);
	put32(out + 52, h->passport);
	put32(out + 56, h->cal_age_ms);
	put16(out + 60, h->spread_cdeg);
	put16(out + 62, h->sync_conf);
	put16(out + 64, (uint16_t)h->gain_tenths_db);
	put16(out + 66, h->mode);
	put32(out + 68, ATKDAQ_PAYLOAD_BYTES(n, h->samples_per_ch));
	put64(out + 72, h->stream_sample0);
	put64(out + 80, h->seg_sample0);
	put32(out + 88, 0);           /* CRC computed over the header with this zero */
	put32(out + 92, 0);
	for (i = 0; i < n; i++) {
		uint8_t *c = out + ATKDAQ_FIXED_BYTES + (size_t)i * ATKDAQ_CH_STRIDE;
		put32(c + 0, (uint32_t)ch[i].delay);
		putf(c + 4, ch[i].frac_delay);
		putf(c + 8, ch[i].w_re);
		putf(c + 12, ch[i].w_im);
		put32(c + 16, ch[i].drops);
		put32(c + 20, ch[i].clip);
		put64(c + 24, ch[i].sample_count);
	}
	crc = atkdaq_crc32(0, out, total);
	put32(out + 88, crc);
	return total;
}

int atkdaq_hdr_unpack(const uint8_t *in, size_t n, atkdaq_hdr *h, atkdaq_ch *ch, int max_ch)
{
	uint16_t hdr_bytes, ch_off, ch_stride;
	uint32_t crc_stored, crc;
	int nch, i;
	uint8_t tmp[4];

	if (n < ATKDAQ_FIXED_BYTES)
		return -1;
	if (memcmp(in, ATKDAQ_MAGIC, 4) != 0)
		return -2;
	/* sizes and CRC before the version: a damaged version field must read
	 * as damage (-4), and only a header that checks can claim to be a
	 * newer format (-3) */
	hdr_bytes = get16(in + 6);
	ch_off = get16(in + 8);
	ch_stride = get16(in + 10);
	nch = in[12];
	if (nch < 1 || nch > ATKDAQ_MAX_CHANNELS || ch_off < ATKDAQ_FIXED_BYTES ||
	    ch_stride < ATKDAQ_CH_STRIDE || hdr_bytes < (uint32_t)ch_off + (uint32_t)nch * ch_stride)
		return -5;
	if (n < hdr_bytes)
		return -1;
	crc_stored = get32(in + 88);
	/* CRC over the header with the CRC field zeroed, without copying it */
	memset(tmp, 0, 4);
	crc = atkdaq_crc32(0, in, 88);
	crc = atkdaq_crc32(crc, tmp, 4);
	crc = atkdaq_crc32(crc, in + 92, (size_t)hdr_bytes - 92);
	if (crc != crc_stored)
		return -4;
	if (get16(in + 4) != ATKDAQ_VERSION)
		return -3;

	memset(h, 0, sizeof(*h));
	memcpy(h->magic, in, 4);
	h->version = get16(in + 4);
	h->hdr_bytes = hdr_bytes;
	h->ch_offset = ch_off;
	h->ch_stride = ch_stride;
	h->n_channels = (uint8_t)nch;
	h->fmt = in[13];
	h->cal_state = in[14];
	h->device_kind = in[15];
	h->flags = get32(in + 16);
	h->seq = get32(in + 20);
	h->t_utc_ns = (int64_t)get64(in + 24);
	h->fc_hz = get64(in + 32);
	h->fs_hz = get32(in + 40);
	h->samples_per_ch = get32(in + 44);
	h->segment = get32(in + 48);
	h->passport = get32(in + 52);
	h->cal_age_ms = get32(in + 56);
	h->spread_cdeg = get16(in + 60);
	h->sync_conf = get16(in + 62);
	h->gain_tenths_db = (int16_t)get16(in + 64);
	h->mode = get16(in + 66);
	h->payload_bytes = get32(in + 68);
	h->stream_sample0 = get64(in + 72);
	h->seg_sample0 = get64(in + 80);
	h->hdr_crc32 = crc_stored;
	if (h->payload_bytes != ATKDAQ_PAYLOAD_BYTES(nch, h->samples_per_ch))
		return -5;
	for (i = 0; i < nch && i < max_ch; i++) {
		const uint8_t *c = in + ch_off + (size_t)i * ch_stride;
		ch[i].delay = (int32_t)get32(c + 0);
		ch[i].frac_delay = getf(c + 4);
		ch[i].w_re = getf(c + 8);
		ch[i].w_im = getf(c + 12);
		ch[i].drops = get32(c + 16);
		ch[i].clip = get32(c + 20);
		ch[i].sample_count = get64(c + 24);
	}
	return (int)hdr_bytes;
}

/* ------------------------------------------------------------------------ */
/* payload conversion                                                        */
/* ------------------------------------------------------------------------ */

uint32_t atkdaq_u8_to_ci8(int8_t *dst, const uint8_t *src, size_t nbytes)
{
	/* One pass: XOR 0x80 turns offset binary into two's complement, and a
	 * byte at 0 or 255 is a sample at the rail. A sample is clipped when
	 * either I or Q is, counted once. */
	uint32_t clip = 0;
	size_t i;
	for (i = 0; i + 1 < nbytes; i += 2) {
		uint8_t a = src[i], b = src[i + 1];
		dst[i] = (int8_t)(a ^ 0x80u);
		dst[i + 1] = (int8_t)(b ^ 0x80u);
		clip += (uint32_t)((a == 0) | (a == 255) | (b == 0) | (b == 255));
	}
	return clip;
}

void atkdaq_u8_to_cf32(float *dst, const uint8_t *src, size_t nbytes)
{
	size_t i;
	for (i = 0; i < nbytes; i++)
		dst[i] = ((float)src[i] - 127.5f) * (1.0f / 127.5f);
}
