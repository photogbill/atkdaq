/* plat.c — see plat.h. */
#include "plat.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  include <fcntl.h>
#  include <io.h>
#  include <process.h>
#else
#  include <errno.h>
#  include <fcntl.h>
#  include <signal.h>
#  include <time.h>
#  include <unistd.h>
#endif

/* ------------------------------------------------------------------------ */
/* threads                                                                   */
/* ------------------------------------------------------------------------ */

typedef struct {
	plat_thread_fn fn;
	void *arg;
} tramp_t;

#ifdef _WIN32
static unsigned __stdcall tramp(void *p)
{
	tramp_t t = *(tramp_t *)p;
	free(p);
	t.fn(t.arg);
	return 0;
}

int plat_thread_start(plat_thread *th, plat_thread_fn fn, void *arg)
{
	uintptr_t h;
	tramp_t *t = (tramp_t *)malloc(sizeof(*t));
	if (!t)
		return -1;
	t->fn = fn;
	t->arg = arg;
	h = _beginthreadex(NULL, 0, tramp, t, 0, NULL);
	if (h == 0) {
		free(t);
		return -1;
	}
	*th = (HANDLE)h;
	return 0;
}

void plat_thread_join(plat_thread t)
{
	WaitForSingleObject(t, INFINITE);
	CloseHandle(t);
}

void plat_mutex_init(plat_mutex *m) { InitializeCriticalSection(m); }
void plat_mutex_destroy(plat_mutex *m) { DeleteCriticalSection(m); }
void plat_mutex_lock(plat_mutex *m) { EnterCriticalSection(m); }
void plat_mutex_unlock(plat_mutex *m) { LeaveCriticalSection(m); }
void plat_cond_init(plat_cond *c) { InitializeConditionVariable(c); }
void plat_cond_destroy(plat_cond *c) { (void)c; }

int plat_cond_wait(plat_cond *c, plat_mutex *m, int timeout_ms)
{
	DWORD ms = timeout_ms < 0 ? INFINITE : (DWORD)timeout_ms;
	if (SleepConditionVariableCS(c, m, ms))
		return 0;
	return 1;
}

void plat_cond_signal(plat_cond *c) { WakeConditionVariable(c); }
void plat_cond_broadcast(plat_cond *c) { WakeAllConditionVariable(c); }

#else /* POSIX */

static void *tramp(void *p)
{
	tramp_t t = *(tramp_t *)p;
	free(p);
	t.fn(t.arg);
	return NULL;
}

int plat_thread_start(plat_thread *th, plat_thread_fn fn, void *arg)
{
	tramp_t *t = (tramp_t *)malloc(sizeof(*t));
	if (!t)
		return -1;
	t->fn = fn;
	t->arg = arg;
	if (pthread_create(th, NULL, tramp, t) != 0) {
		free(t);
		return -1;
	}
	return 0;
}

void plat_thread_join(plat_thread t) { pthread_join(t, NULL); }

void plat_mutex_init(plat_mutex *m) { pthread_mutex_init(m, NULL); }
void plat_mutex_destroy(plat_mutex *m) { pthread_mutex_destroy(m); }
void plat_mutex_lock(plat_mutex *m) { pthread_mutex_lock(m); }
void plat_mutex_unlock(plat_mutex *m) { pthread_mutex_unlock(m); }

void plat_cond_init(plat_cond *c)
{
	pthread_condattr_t a;
	pthread_condattr_init(&a);
	pthread_condattr_setclock(&a, CLOCK_MONOTONIC);
	pthread_cond_init(c, &a);
	pthread_condattr_destroy(&a);
}

void plat_cond_destroy(plat_cond *c) { pthread_cond_destroy(c); }

int plat_cond_wait(plat_cond *c, plat_mutex *m, int timeout_ms)
{
	struct timespec ts;
	if (timeout_ms < 0)
		return pthread_cond_wait(c, m) == 0 ? 0 : 1;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	ts.tv_sec += timeout_ms / 1000;
	ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
	if (ts.tv_nsec >= 1000000000L) {
		ts.tv_sec += 1;
		ts.tv_nsec -= 1000000000L;
	}
	return pthread_cond_timedwait(c, m, &ts) == 0 ? 0 : 1;
}

void plat_cond_signal(plat_cond *c) { pthread_cond_signal(c); }
void plat_cond_broadcast(plat_cond *c) { pthread_cond_broadcast(c); }
#endif

/* ------------------------------------------------------------------------ */
/* time                                                                      */
/* ------------------------------------------------------------------------ */

#ifdef _WIN32
int64_t plat_mono_ns(void)
{
	static LARGE_INTEGER freq;
	LARGE_INTEGER now;
	if (freq.QuadPart == 0)
		QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&now);
	/* split to avoid overflow: whole seconds, then the remainder */
	return (int64_t)(now.QuadPart / freq.QuadPart) * 1000000000LL +
	       (int64_t)((now.QuadPart % freq.QuadPart) * 1000000000LL / freq.QuadPart);
}

typedef VOID (WINAPI *precise_fn)(LPFILETIME);

int64_t plat_utc_ns(void)
{
	/* GetSystemTimePreciseAsFileTime is Windows 8+; resolved at run time so
	 * the binary still starts (with 1-16 ms resolution) on anything older. */
	static precise_fn precise;
	static int looked;
	FILETIME ft;
	ULARGE_INTEGER u;
	if (!looked) {
		HMODULE k = GetModuleHandleA("kernel32.dll");
		if (k)
			precise = (precise_fn)(void *)GetProcAddress(k, "GetSystemTimePreciseAsFileTime");
		looked = 1;
	}
	if (precise)
		precise(&ft);
	else
		GetSystemTimeAsFileTime(&ft);
	u.LowPart = ft.dwLowDateTime;
	u.HighPart = ft.dwHighDateTime;
	/* 100 ns ticks since 1601-01-01 -> ns since 1970-01-01 */
	return ((int64_t)u.QuadPart - 116444736000000000LL) * 100LL;
}

void plat_sleep_ms(int ms) { Sleep(ms > 0 ? (DWORD)ms : 0); }

#else

int64_t plat_mono_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

int64_t plat_utc_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

void plat_sleep_ms(int ms)
{
	struct timespec ts;
	if (ms <= 0)
		return;
	ts.tv_sec = ms / 1000;
	ts.tv_nsec = (long)(ms % 1000) * 1000000L;
	while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {
	}
}
#endif

void plat_sleep_until_ns(int64_t t_ns)
{
	for (;;) {
		int64_t now = plat_mono_ns();
		int64_t left = t_ns - now;
		if (left <= 0)
			return;
		if (left > 2000000LL)
			plat_sleep_ms((int)((left - 1000000LL) / 1000000LL));
		else
			plat_sleep_ms(1);
	}
}

/* ------------------------------------------------------------------------ */
/* stdio                                                                     */
/* ------------------------------------------------------------------------ */

void plat_binary_stdio(void)
{
#ifdef _WIN32
	_setmode(_fileno(stdout), _O_BINARY);
	_setmode(_fileno(stdin), _O_BINARY);
#endif
}

void plat_ignore_sigpipe(void)
{
#ifndef _WIN32
	signal(SIGPIPE, SIG_IGN);
#endif
}

struct plat_out {
#ifdef _WIN32
	HANDLE h;
#else
	int fd;
#endif
	int owned;
	int discard;
};

static plat_out g_null_out;

plat_out *plat_out_null(void)
{
	g_null_out.discard = 1;
	g_null_out.owned = 0;
	return &g_null_out;
}

static plat_out g_stdout_out;

plat_out *plat_out_stdout(void)
{
	fflush(stdout);
#ifdef _WIN32
	g_stdout_out.h = GetStdHandle(STD_OUTPUT_HANDLE);
#else
	g_stdout_out.fd = 1;
#endif
	g_stdout_out.owned = 0;
	return &g_stdout_out;
}

plat_out *plat_out_open_file(const char *path)
{
	plat_out *o = (plat_out *)calloc(1, sizeof(*o));
	if (!o)
		return NULL;
#ifdef _WIN32
	o->h = CreateFileA(path, GENERIC_WRITE, FILE_SHARE_READ, NULL, CREATE_ALWAYS,
	                   FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, NULL);
	if (o->h == INVALID_HANDLE_VALUE) {
		free(o);
		return NULL;
	}
#else
	o->fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (o->fd < 0) {
		free(o);
		return NULL;
	}
#endif
	o->owned = 1;
	return o;
}

int plat_out_write(plat_out *o, const void *buf, size_t n)
{
	const unsigned char *p = (const unsigned char *)buf;
	if (o->discard)
		return 0;
	while (n > 0) {
#ifdef _WIN32
		DWORD chunk = n > 0x40000000u ? 0x40000000u : (DWORD)n;
		DWORD wrote = 0;
		if (!WriteFile(o->h, p, chunk, &wrote, NULL))
			return -1;
		if (wrote == 0)
			return -1;
#else
		ssize_t wrote = write(o->fd, p, n);
		if (wrote < 0) {
			if (errno == EINTR)
				continue;
			return -1;
		}
		if (wrote == 0)
			return -1;
#endif
		p += wrote;
		n -= (size_t)wrote;
	}
	return 0;
}

void plat_out_close(plat_out *o)
{
	if (!o || !o->owned)
		return;
#ifdef _WIN32
	CloseHandle(o->h);
#else
	close(o->fd);
#endif
	free(o);
}

/* Raw OS reads, never stdio: a thread blocked in fgetc() HOLDS stdin's
 * stdio lock, and exit() walks every stdio stream on the way out — so a
 * program that finished on its own (a --seconds limit) while its command
 * thread waited for input hung in exit() for ever (found by the throughput
 * bench). A blocked read(0) / ReadFile holds nothing and dies with the
 * process. */
static char g_in[4096];
static int g_in_len, g_in_pos, g_in_eof;

static int in_fill(void)
{
#ifdef _WIN32
	DWORD got = 0;
	if (!ReadFile(GetStdHandle(STD_INPUT_HANDLE), g_in, sizeof(g_in), &got, NULL) || got == 0)
		return -1;
	g_in_len = (int)got;
#else
	ssize_t got;
	do {
		got = read(0, g_in, sizeof(g_in));
	} while (got < 0 && errno == EINTR);
	if (got <= 0)
		return -1;
	g_in_len = (int)got;
#endif
	g_in_pos = 0;
	return 0;
}

int plat_read_line(char *buf, int cap)
{
	int n = 0;
	if (cap <= 1)
		return -1;
	for (;;) {
		char c;
		if (g_in_pos >= g_in_len) {
			if (g_in_eof || in_fill() != 0) {
				g_in_eof = 1;
				buf[n] = 0;
				return n > 0 ? n : -1;
			}
		}
		c = g_in[g_in_pos++];
		if (c == '\n')
			break;
		if (c == '\r')
			continue;
		if (n < cap - 1)
			buf[n++] = c;
	}
	buf[n] = 0;
	return n;
}

int plat_stdin_is_console(void)
{
#ifdef _WIN32
	DWORD mode;
	return GetConsoleMode(GetStdHandle(STD_INPUT_HANDLE), &mode) ? 1 : 0;
#else
	return isatty(0);
#endif
}
