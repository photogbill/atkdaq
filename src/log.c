/* log.c — see log.h. */
#include "log.h"

#include <stdarg.h>
#include <stdio.h>

#include "plat.h"

int log_verbose = 0;
static plat_mutex g_lock;
static int g_ready;

void log_init(void)
{
	if (!g_ready) {
		plat_mutex_init(&g_lock);
		g_ready = 1;
	}
}

void log_event(const char *event, const char *fmt, ...)
{
	char line[2048];
	int n;
	va_list ap;
	n = snprintf(line, sizeof(line), "ATKDAQ event=%s ", event);
	if (n < 0 || n >= (int)sizeof(line))
		return;
	va_start(ap, fmt);
	vsnprintf(line + n, sizeof(line) - (size_t)n, fmt, ap);
	va_end(ap);
	if (g_ready)
		plat_mutex_lock(&g_lock);
	fputs(line, stderr);
	fputc('\n', stderr);
	fflush(stderr);
	if (g_ready)
		plat_mutex_unlock(&g_lock);
}
