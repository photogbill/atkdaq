/*
 * plat.h — the only place atkdaq knows which operating system it is on.
 *
 * Threads, locks, time, sleeping and writing a binary stream. Windows is the
 * target; POSIX exists so the whole program — including the synthetic device
 * and the end-to-end tests — builds and runs on the Linux machines the
 * development sessions use. librtlsdr has its own shim (vendor/win32); this
 * layer is for atkdaq's own threads and never touches that one.
 */
#ifndef ATKDAQ_PLAT_H
#define ATKDAQ_PLAT_H

#include <stddef.h>
#include <stdint.h>

#ifdef _WIN32
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
typedef HANDLE              plat_thread;
typedef CRITICAL_SECTION    plat_mutex;
typedef CONDITION_VARIABLE  plat_cond;
#else
#  include <pthread.h>
typedef pthread_t           plat_thread;
typedef pthread_mutex_t     plat_mutex;
typedef pthread_cond_t      plat_cond;
#endif

typedef void (*plat_thread_fn)(void *arg);

int  plat_thread_start(plat_thread *t, plat_thread_fn fn, void *arg);
void plat_thread_join(plat_thread t);

void plat_mutex_init(plat_mutex *m);
void plat_mutex_destroy(plat_mutex *m);
void plat_mutex_lock(plat_mutex *m);
void plat_mutex_unlock(plat_mutex *m);

void plat_cond_init(plat_cond *c);
void plat_cond_destroy(plat_cond *c);
/* Waits up to timeout_ms (negative = forever) with m held; returns 0 when
 * signalled, 1 on timeout. Spurious wake-ups are possible: always re-check. */
int  plat_cond_wait(plat_cond *c, plat_mutex *m, int timeout_ms);
void plat_cond_signal(plat_cond *c);
void plat_cond_broadcast(plat_cond *c);

/* Monotonic nanoseconds (QueryPerformanceCounter / CLOCK_MONOTONIC). */
int64_t plat_mono_ns(void);
/* UTC nanoseconds since the Unix epoch (GetSystemTimePreciseAsFileTime /
 * CLOCK_REALTIME). */
int64_t plat_utc_ns(void);
void    plat_sleep_ms(int ms);
/* Sleep until the monotonic clock reaches t_ns (returns at once if past). */
void    plat_sleep_until_ns(int64_t t_ns);

/* Put stdout (and stdin) into binary mode. MUST run before the first byte of
 * a frame: on Windows, text mode rewrites every 0x0A in the payload. */
void plat_binary_stdio(void);

/* Write all n bytes to an OS file handle/descriptor, looping over partial
 * writes. Returns 0, or -1 when the reader has gone (broken pipe) or on any
 * other error. On Windows one frame goes out as one WriteFile call when the
 * pipe accepts it whole. */
typedef struct plat_out plat_out;
plat_out *plat_out_stdout(void);
plat_out *plat_out_open_file(const char *path);   /* NULL on failure */
plat_out *plat_out_null(void);                    /* discards everything (the probe) */
int       plat_out_write(plat_out *o, const void *buf, size_t n);
void      plat_out_close(plat_out *o);

/* Line-at-a-time reading of stdin on a dedicated thread. Returns the line
 * length (without the newline), 0 for an empty line, or -1 at end of input
 * (the consumer closed our stdin — which means it has gone). */
int plat_read_line(char *buf, int cap);

/* 1 if stdin is interactive (a console), so a human typing commands gets the
 * same parser without the program assuming a pipe. */
int plat_stdin_is_console(void);

/* Process-wide: ignore SIGPIPE on POSIX so a closed pipe is an error return,
 * not a silent death. No-op on Windows. */
void plat_ignore_sigpipe(void);

#endif /* ATKDAQ_PLAT_H */
