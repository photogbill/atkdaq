/*
 * log.h — status lines on stderr. Frames go to stdout and NOTHING else does.
 *
 * Every line is machine-readable and human-readable at once:
 *
 *     ATKDAQ event=<name> key=value key=value ...
 *
 * Values never contain spaces unless quoted with "..." (msg= always is).
 * python/atkdaq/client.py parses these; the README lists every event.
 * Lines are written whole under a lock, so threads never interleave them.
 */
#ifndef ATKDAQ_LOG_H
#define ATKDAQ_LOG_H

void log_init(void);
/* fmt produces the key=value part; event is the name. */
void log_event(const char *event, const char *fmt, ...)
#if defined(__GNUC__)
	__attribute__((format(printf, 2, 3)))
#endif
	;
/* 1 while stderr is being written for a person rather than a program
 * (--verbose); only changes how much is said, never the format. */
extern int log_verbose;

#endif
