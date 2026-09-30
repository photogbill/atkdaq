/*
 * ring.c — a power-of-two byte ring with absolute positions.
 *
 * One producer (a USB callback) and one consumer (the frame assembler).
 * Positions are 64-bit byte counts since the stream started, never wrapped,
 * so "which bytes" is a number rather than an index that means different
 * things before and after a wrap — which is what lets the assembler address
 * each channel by its own raw sample count plus a delay.
 *
 * THE PRODUCER NEVER WAITS. A USB callback that blocks stops the device's
 * transfers and loses samples on that channel alone, which is precisely the
 * misalignment this program exists to prevent. So a full ring is overwritten,
 * and the consumer, which checks, finds out that it fell behind and skips
 * every channel to the same aligned position — losing time, never alignment.
 */
#include <stdlib.h>
#include <string.h>

#include "atkdaq_core.h"
#include "atomics.h"

int atkdaq_ring_init(atkdaq_ring *r, uint64_t size_pow2)
{
	memset(r, 0, sizeof(*r));
	if (size_pow2 < 4096 || (size_pow2 & (size_pow2 - 1)) != 0)
		return -1;
	r->buf = (uint8_t *)malloc((size_t)size_pow2);
	if (!r->buf)
		return -2;
	r->size = size_pow2;
	r->mask = size_pow2 - 1;
	return 0;
}

void atkdaq_ring_free(atkdaq_ring *r)
{
	free(r->buf);
	memset(r, 0, sizeof(*r));
}

void atkdaq_ring_write(atkdaq_ring *r, const uint8_t *src, size_t n)
{
	uint64_t w = r->wpos;     /* only this thread writes wpos */
	while (n > 0) {
		uint64_t off = w & r->mask;
		size_t chunk = (size_t)(r->size - off);
		if (chunk > n)
			chunk = n;
		memcpy(r->buf + off, src, chunk);
		src += chunk;
		n -= chunk;
		w += chunk;
	}
	atkq_store_rel(&r->wpos, w);
}

uint64_t atkdaq_ring_wpos(const atkdaq_ring *r)
{
	return atkq_load_acq(&r->wpos);
}

uint64_t atkdaq_ring_oldest(const atkdaq_ring *r)
{
	uint64_t w = atkq_load_acq(&r->wpos);
	return w > r->size ? w - r->size : 0;
}

/* Copy [pos, pos+n) out of the ring into dst using fn (a plain copy or the
 * int8 conversion). Returns 0/-1/-2 as documented in atkdaq_core.h. */
typedef void (*copy_fn)(void *dst, const uint8_t *src, size_t n, uint32_t *acc);

static void plain_copy(void *dst, const uint8_t *src, size_t n, uint32_t *acc)
{
	(void)acc;
	memcpy(dst, src, n);
}

static void ci8_copy(void *dst, const uint8_t *src, size_t n, uint32_t *acc)
{
	*acc += atkdaq_u8_to_ci8((int8_t *)dst, src, n);
}

static int ring_read_with(const atkdaq_ring *r, uint64_t pos, void *dst, size_t n,
                          copy_fn fn, uint32_t *acc)
{
	uint64_t w = atkq_load_acq(&r->wpos);
	uint64_t p = pos;
	uint8_t *d = (uint8_t *)dst;
	size_t left = n;
	if (n > r->size)
		return -3;
	if (pos + n > w)
		return -1;
	if (w > r->size && pos < w - r->size)
		return -2;
	while (left > 0) {
		uint64_t off = p & r->mask;
		size_t chunk = (size_t)(r->size - off);
		if (chunk > left)
			chunk = left;
		fn(d, r->buf + off, chunk, acc);
		d += chunk;
		p += chunk;
		left -= chunk;
	}
	/* Did the producer overwrite any of it while we copied? It may have
	 * advanced by anything; if the oldest byte it could have touched is now
	 * past `pos`, part of the copy may be new data wearing old positions. */
	w = atkq_load_acq(&r->wpos);
	if (w > r->size && pos < w - r->size)
		return -2;
	return 0;
}

int atkdaq_ring_read(const atkdaq_ring *r, uint64_t pos, uint8_t *dst, size_t n)
{
	return ring_read_with(r, pos, dst, n, plain_copy, NULL);
}

int atkdaq_ring_read_ci8(const atkdaq_ring *r, uint64_t pos, int8_t *dst, size_t n, uint32_t *clip)
{
	uint32_t acc = 0;
	int rc;
	if ((pos & 1u) || (n & 1u))
		return -3;          /* I/Q pairs only; an odd position is a caller bug */
	rc = ring_read_with(r, pos, dst, n, ci8_copy, &acc);
	if (rc == 0 && clip)
		*clip += acc;
	return rc;
}

void atkdaq_ring_set_floor(atkdaq_ring *r, uint64_t pos)
{
	atkq_store_rel(&r->floor, pos);
}

uint64_t atkdaq_ring_free_space(const atkdaq_ring *r)
{
	uint64_t used = atkq_load_acq(&r->wpos) - atkq_load_acq(&r->floor);
	return used >= r->size ? 0 : r->size - used;
}
