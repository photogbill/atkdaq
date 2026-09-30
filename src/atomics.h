/*
 * atomics.h — the two memory orderings the rings need, and nothing else.
 *
 * One producer thread publishes bytes and then a 64-bit position; one
 * consumer reads the position and then the bytes. That is release on the
 * store and acquire on the load. On x86-64 every aligned 64-bit load and
 * store is already atomic and ordered that way at the hardware level, so on
 * MSVC the only job is to stop the COMPILER reordering around them, which
 * _ReadWriteBarrier does. GCC and Clang get the real builtins.
 *
 * atkdaq targets x86-64 Windows. An ARM64 Windows build would need real
 * load-acquire/store-release instructions here (__ldar64/__stlr64); that is
 * refused at compile time rather than left to produce a torn ring.
 */
#ifndef ATKDAQ_ATOMICS_H
#define ATKDAQ_ATOMICS_H

#include <stdint.h>

#if defined(_MSC_VER) && !defined(__clang__)
#  if !defined(_M_X64) && !defined(_M_IX86)
#    error "atkdaq's ring atomics are written for x86/x64; see atomics.h"
#  endif
#  include <intrin.h>
static __inline uint64_t atkq_load_acq(const volatile uint64_t *p)
{
	uint64_t v = *p;
	_ReadWriteBarrier();
	return v;
}
static __inline void atkq_store_rel(volatile uint64_t *p, uint64_t v)
{
	_ReadWriteBarrier();
	*p = v;
}
static __inline uint32_t atkq_load32_acq(const volatile uint32_t *p)
{
	uint32_t v = *p;
	_ReadWriteBarrier();
	return v;
}
static __inline void atkq_store32_rel(volatile uint32_t *p, uint32_t v)
{
	_ReadWriteBarrier();
	*p = v;
}
#else
static inline uint64_t atkq_load_acq(const volatile uint64_t *p)
{
	return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}
static inline void atkq_store_rel(volatile uint64_t *p, uint64_t v)
{
	__atomic_store_n(p, v, __ATOMIC_RELEASE);
}
static inline uint32_t atkq_load32_acq(const volatile uint32_t *p)
{
	return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}
static inline void atkq_store32_rel(volatile uint32_t *p, uint32_t v)
{
	__atomic_store_n(p, v, __ATOMIC_RELEASE);
}
#endif

#endif /* ATKDAQ_ATOMICS_H */
