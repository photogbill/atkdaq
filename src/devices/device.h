/*
 * device.h — what a coherent multi-channel receiver must do for atkdaq.
 *
 * Two implementations: kraken.c (the KrakenSDR, through the krakenrf
 * librtlsdr fork) and synth.c (a synthetic coherent receiver with planted
 * delays, phases, fractional delays, drops and a switchable noise source —
 * the device every test runs against). The vtable is what a second REAL
 * coherent device would implement; it is not generalised further than that
 * until one exists.
 *
 * THREADING. start() spawns the device's own reader threads; they call
 * on_data from those threads, never from the caller's. Every other call is
 * made from atkdaq's assembler thread only. on_data must not block and must
 * not call back into the device (librtlsdr forbids calling it from its own
 * callback, and the ring it writes never waits).
 *
 * WHERE A CHANGE TAKES EFFECT. noise/tune/gain return the channel-0 raw
 * sample index from which the change is certainly in force, when the device
 * knows it (the synthetic device always does), or ATKDAQ_UNKNOWN_INDEX, in
 * which case atkdaq takes the count received so far as the boundary and
 * relies on the settle window. Flags on frames are decided by sample index,
 * never by wall clock, so the synthetic device can run faster than real time
 * and every decision stays the same.
 */
#ifndef ATKDAQ_DEVICE_H
#define ATKDAQ_DEVICE_H

#include <stddef.h>
#include <stdint.h>

#include "../config.h"

struct atkdaq_ring;

#define ATKDAQ_UNKNOWN_INDEX UINT64_MAX

typedef struct atkdaq_dev atkdaq_dev;

/* One USB transfer's worth (or the synthetic equivalent) for channel ch:
 * len raw bytes (unsigned 8-bit I,Q), and the monotonic time it arrived. */
typedef void (*atkdaq_on_data)(void *user, int ch, const uint8_t *buf, uint32_t len,
                               int64_t t_arrival_ns);
/* A reader thread ended on its own (device unplugged, USB error). */
typedef void (*atkdaq_on_fault)(void *user, int ch, const char *what);

typedef struct atkdaq_dev_ops {
	const char *name;
	int kind;                                     /* ATKDAQ_DEV_* */
	int (*open)(atkdaq_dev **out, const atkdaq_config *cfg, char *err, int errlen);
	void (*close)(atkdaq_dev *d);
	int (*start)(atkdaq_dev *d, atkdaq_on_data on_data, atkdaq_on_fault on_fault, void *user);
	void (*stop)(atkdaq_dev *d);
	int (*tune)(atkdaq_dev *d, uint64_t fc_hz, uint64_t *n_effective);
	int (*set_gain)(atkdaq_dev *d, int tenths_db, int *applied_tenths, uint64_t *n_effective);
	int (*noise)(atkdaq_dev *d, int on, uint64_t *n_effective);
	/* 1 = every tuner reports lock, 0 = some tuner does not, -1 = cannot tell */
	int (*pll_locked)(atkdaq_dev *d);
	/* The device's notion of "now" in the timebase of its arrival stamps. */
	int64_t (*now_ns)(atkdaq_dev *d);
	/* One line describing the hardware, for the status stream and the probe. */
	void (*describe)(atkdaq_dev *d, char *buf, int len);
	/* Supported gains in tenths of dB; returns the count (<= max). */
	int (*gains)(atkdaq_dev *d, int *list, int max);
	/* Flow control: the rings a device MAY wait on before writing (the
	 * synthetic device in fast mode). Set by atkdaq before start(). A real
	 * receiver never waits and implements this as a no-op. */
	void (*set_rings)(atkdaq_dev *d, struct atkdaq_ring *const *rings, int n);
} atkdaq_dev_ops;

extern const atkdaq_dev_ops atkdaq_kraken_ops;
extern const atkdaq_dev_ops atkdaq_synth_ops;

/* Enumerate RTL devices (the probe and `atkdaq enum`); prints to stdout.
 * Returns the number found, or -1 when this build has no librtlsdr. */
int atkdaq_kraken_enumerate(void);

#endif /* ATKDAQ_DEVICE_H */
