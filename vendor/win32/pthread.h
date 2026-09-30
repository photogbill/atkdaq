/*
 * pthread.h for building the krakenrf librtlsdr fork on Windows WITHOUT a
 * pthreads library. atkdaq's own file (not part of librtlsdr). build.bat and
 * CMakeLists.txt put vendor\win32 first on the include path on Windows only,
 * so librtlsdr.c's `#include <pthread.h>` lands here.
 *
 * WHY: librtlsdr's Windows build expects pthreads-win32, which would be one
 * more third-party DLL to fetch, version and license. ATK's get_hackrf.bat
 * made the same decision for libhackrf (tools\hackrf_build\pthread.h, built
 * and run on Bill's machine 2026-09-25); this is that shim's sibling.
 *
 * WHAT IS DIFFERENT FROM ATK'S HACKRF SHIM, and why it matters:
 *
 *   * The mutex is a CRITICAL_SECTION, not an SRW lock. librtlsdr creates
 *     its I2C/critical-section lock with PTHREAD_MUTEX_RECURSIVE
 *     (`dev->cs_mutex`, librtlsdr.c) and re-enters it; an SRW lock
 *     deadlocks on re-entry. A CRITICAL_SECTION is recursive by
 *     construction, so the attribute can be accepted and ignored.
 *   * pthread_attr_* and pthread_exit exist, because the soft-AGC worker
 *     thread uses them. atkdaq never enables soft AGC (AGC must be OFF for
 *     coherence), but the code is compiled, so the calls must link.
 *
 * The complete list of pthread identifiers librtlsdr.c uses when
 * WITH_UDP_SERVER is not defined (grepped from the pinned commit, see
 * vendor/librtlsdr/PINNED.txt): pthread_t, pthread_attr_t,
 * pthread_mutex_t, pthread_mutexattr_t, pthread_cond_t, pthread_create,
 * pthread_join, pthread_exit, pthread_attr_init, pthread_attr_destroy,
 * pthread_attr_setdetachstate, pthread_mutexattr_init,
 * pthread_mutexattr_settype, pthread_mutex_init/lock/unlock/destroy,
 * pthread_cond_init/wait/signal/destroy, PTHREAD_CREATE_JOINABLE,
 * PTHREAD_MUTEX_RECURSIVE. Nothing else is provided, on purpose: a call
 * that is not here should fail to COMPILE, not quietly misbehave.
 *
 * Checked by cross-compiling librtlsdr + this header with MinGW-w64
 * (tests/check_windows_build.sh) — MinGW ships its own pthread.h, and the
 * include order puts this one first, so the check exercises this file.
 */
#ifndef ATKDAQ_PTHREAD_SHIM_H
#define ATKDAQ_PTHREAD_SHIM_H

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>   /* before windows.h, as librtlsdr includes it too */
#include <windows.h>
#include <process.h>
#include <stdlib.h>
#include <errno.h>

typedef struct atkdaq_pthread {
	HANDLE handle;
	void* (*start)(void*);
	void* arg;
	void* result;
} *pthread_t;

typedef CRITICAL_SECTION pthread_mutex_t;
typedef CONDITION_VARIABLE pthread_cond_t;
typedef struct { int detach; } pthread_attr_t;
typedef struct { int type; } pthread_mutexattr_t;
typedef void pthread_condattr_t;

#define PTHREAD_CREATE_JOINABLE 0
#define PTHREAD_CREATE_DETACHED 1
#define PTHREAD_MUTEX_NORMAL    0
#define PTHREAD_MUTEX_RECURSIVE 1

static unsigned __stdcall atkdaq_pthread_trampoline(void* p)
{
	pthread_t t = (pthread_t) p;
	t->result = t->start(t->arg);
	return 0;
}

static __inline int pthread_attr_init(pthread_attr_t* a)
{
	a->detach = PTHREAD_CREATE_JOINABLE;
	return 0;
}

static __inline int pthread_attr_destroy(pthread_attr_t* a)
{
	(void) a;
	return 0;
}

static __inline int pthread_attr_setdetachstate(pthread_attr_t* a, int state)
{
	/* Only joinable threads are supported; librtlsdr only asks for those. */
	if (state != PTHREAD_CREATE_JOINABLE)
		return EINVAL;
	a->detach = state;
	return 0;
}

static __inline int pthread_create(
	pthread_t* thread,
	const pthread_attr_t* attr,
	void* (*start)(void*),
	void* arg)
{
	uintptr_t h;
	pthread_t t;
	(void) attr;
	t = (pthread_t) calloc(1, sizeof(*t));
	if (t == NULL)
		return ENOMEM;
	t->start = start;
	t->arg = arg;
	h = _beginthreadex(NULL, 0, atkdaq_pthread_trampoline, t, 0, NULL);
	if (h == 0) {
		free(t);
		return EAGAIN;
	}
	t->handle = (HANDLE) h;
	*thread = t;
	return 0;
}

static __inline int pthread_join(pthread_t thread, void** result)
{
	if (thread == NULL)
		return EINVAL;
	if (WaitForSingleObject(thread->handle, INFINITE) != WAIT_OBJECT_0)
		return EINVAL;
	CloseHandle(thread->handle);
	if (result != NULL)
		*result = thread->result;
	free(thread);
	return 0;
}

/* The thread's `result` stays NULL: librtlsdr only ever exits with 0 and
 * joins with a NULL result pointer. */
static __inline void pthread_exit(void* value)
{
	(void) value;
	_endthreadex(0);
}

static __inline int pthread_mutexattr_init(pthread_mutexattr_t* a)
{
	a->type = PTHREAD_MUTEX_NORMAL;
	return 0;
}

static __inline int pthread_mutexattr_settype(pthread_mutexattr_t* a, int type)
{
	/* Recorded and ignored: a CRITICAL_SECTION is always recursive, which
	 * satisfies both types for every use librtlsdr makes of them. */
	a->type = type;
	return 0;
}

static __inline int pthread_mutexattr_destroy(pthread_mutexattr_t* a)
{
	(void) a;
	return 0;
}

static __inline int pthread_mutex_init(pthread_mutex_t* m, const pthread_mutexattr_t* a)
{
	(void) a;
	InitializeCriticalSection(m);
	return 0;
}

static __inline int pthread_mutex_destroy(pthread_mutex_t* m)
{
	DeleteCriticalSection(m);
	return 0;
}

static __inline int pthread_mutex_lock(pthread_mutex_t* m)
{
	EnterCriticalSection(m);
	return 0;
}

static __inline int pthread_mutex_unlock(pthread_mutex_t* m)
{
	LeaveCriticalSection(m);
	return 0;
}

static __inline int pthread_cond_init(pthread_cond_t* c, const pthread_condattr_t* a)
{
	(void) a;
	InitializeConditionVariable(c);
	return 0;
}

static __inline int pthread_cond_destroy(pthread_cond_t* c)
{
	(void) c; /* a condition variable owns no resources */
	return 0;
}

static __inline int pthread_cond_wait(pthread_cond_t* c, pthread_mutex_t* m)
{
	return SleepConditionVariableCS(c, m, INFINITE) ? 0 : EINVAL;
}

static __inline int pthread_cond_signal(pthread_cond_t* c)
{
	WakeConditionVariable(c);
	return 0;
}

static __inline int pthread_cond_broadcast(pthread_cond_t* c)
{
	WakeAllConditionVariable(c);
	return 0;
}

#endif /* ATKDAQ_PTHREAD_SHIM_H */
