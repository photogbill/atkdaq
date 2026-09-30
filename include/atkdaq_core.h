/*
 * atkdaq_core.h — the computing core of atkdaq, as a library.
 *
 * NOT the wire format (that is atkdaq_frame.h) and not a public ABI for
 * consumers: these are the functions atkdaq.exe is built from, exported from
 * a small shared library (atkdaq_core.dll / .so) for one reason — so
 * tests/test_cross_check.py can hand the SAME synthetic data to the C and to
 * the numpy twins in python/atkdaq/reference.py and require them to agree.
 * That is atkdsp's rule carried over: every algorithm the C implements has a
 * numpy twin, and a change to one is a change to both, or the tests say so.
 *
 * Nothing here touches a device, a thread or a file.
 */
#ifndef ATKDAQ_CORE_H
#define ATKDAQ_CORE_H

#include <stddef.h>
#include <stdint.h>

#include "atkdaq_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) && defined(ATKDAQ_CORE_SHARED)
#  ifdef ATKDAQ_CORE_BUILD
#    define ATKDAQ_API __declspec(dllexport)
#  else
#    define ATKDAQ_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) && defined(ATKDAQ_CORE_SHARED)
#  define ATKDAQ_API __attribute__((visibility("default")))
#else
#  define ATKDAQ_API
#endif

/* Bumped whenever a function below changes signature or meaning; the ctypes
 * binding refuses a library that disagrees. */
#define ATKDAQ_CORE_ABI 1

ATKDAQ_API int         atkdaq_core_abi(void);
ATKDAQ_API const char *atkdaq_version(void);

/* ------------------------------------------------------------------------ */
/* frame.c — the header, the CRC, the payload conversion                    */
/* ------------------------------------------------------------------------ */

ATKDAQ_API uint32_t atkdaq_crc32(uint32_t crc, const void *data, size_t n);

/* Serialise a header (fixed part + n channel records) into out, computing
 * hdr_bytes, ch_offset, ch_stride, payload_bytes and the CRC. Returns the
 * number of bytes written, or 0 if cap is too small or n is out of range. */
ATKDAQ_API size_t atkdaq_hdr_pack(uint8_t *out, size_t cap, const atkdaq_hdr *h,
                                  const atkdaq_ch *ch, int n);

/* Parse and validate. Returns hdr_bytes (> 0) on success, or:
 *   -1 need more bytes   -2 bad magic   -3 unsupported version
 *   -4 CRC mismatch      -5 inconsistent sizes / too many channels        */
ATKDAQ_API int atkdaq_hdr_unpack(const uint8_t *in, size_t n, atkdaq_hdr *h,
                                 atkdaq_ch *ch, int max_ch);

/* raw RTL bytes (unsigned, offset 128) -> int8, counting samples whose I or
 * Q sits at the ADC rail (raw 0 or 255). nbytes is 2 x samples. Returns the
 * clip count. */
ATKDAQ_API uint32_t atkdaq_u8_to_ci8(int8_t *dst, const uint8_t *src, size_t nbytes);

/* raw RTL bytes -> interleaved float I,Q: (raw - 127.5) / 127.5. */
ATKDAQ_API void atkdaq_u8_to_cf32(float *dst, const uint8_t *src, size_t nbytes);

/* ------------------------------------------------------------------------ */
/* ring.c — one producer, one consumer, bytes addressed by absolute position */
/* ------------------------------------------------------------------------ */

typedef struct atkdaq_ring {
	uint8_t *buf;
	uint64_t size;              /* power of two */
	uint64_t mask;
	volatile uint64_t wpos;     /* bytes ever written; published with release */
	volatile uint64_t floor;    /* producer never waits for the consumer, but a
	                               flow-controlled producer (the synthetic device
	                               in fast mode) waits until wpos - floor < size */
} atkdaq_ring;

ATKDAQ_API int      atkdaq_ring_init(atkdaq_ring *r, uint64_t size_pow2);
ATKDAQ_API void     atkdaq_ring_free(atkdaq_ring *r);
/* Producer: append n bytes, overwriting the oldest data if the ring is full.
 * Never blocks, never fails — a USB callback must not wait. */
ATKDAQ_API void     atkdaq_ring_write(atkdaq_ring *r, const uint8_t *src, size_t n);
ATKDAQ_API uint64_t atkdaq_ring_wpos(const atkdaq_ring *r);
/* Consumer: the bytes at absolute positions [pos, pos+n).
 *   0 copied     -1 not written yet     -2 already overwritten (lost)
 *  -3 invalid request (longer than the ring, or an odd position for ci8)
 * A copy that raced the producer's overwrite is detected afterwards and
 * reported as -2, so a torn copy is never returned as data. */
ATKDAQ_API int      atkdaq_ring_read(const atkdaq_ring *r, uint64_t pos, uint8_t *dst, size_t n);
/* The same, converting to int8 and counting clips on the way (no second pass). */
ATKDAQ_API int      atkdaq_ring_read_ci8(const atkdaq_ring *r, uint64_t pos, int8_t *dst,
                                         size_t n, uint32_t *clip);
/* The oldest position still guaranteed readable. */
ATKDAQ_API uint64_t atkdaq_ring_oldest(const atkdaq_ring *r);
/* Flow control for producers that CAN wait (the synthetic device in fast
 * mode, a file): the consumer publishes the oldest position it still needs,
 * and the producer checks the free space before writing. A real device
 * never waits and never calls this. */
ATKDAQ_API void     atkdaq_ring_set_floor(atkdaq_ring *r, uint64_t pos);
ATKDAQ_API uint64_t atkdaq_ring_free_space(const atkdaq_ring *r);

/* ------------------------------------------------------------------------ */
/* clock.c — when was raw sample n digitised?                               */
/* ------------------------------------------------------------------------ */

#define ATKDAQ_SCLOCK_WIN 128

/* Each USB transfer arrives some time AFTER its last sample was digitised,
 * and that latency is never negative. So for arrival t_j of a transfer
 * ending at sample n_j, t_j - n_j*P >= a (the instant sample 0 was taken,
 * plus the minimum latency), with equality for the luckiest transfer. The
 * lower envelope of those residuals over a window is therefore a far better
 * clock than any single arrival — the same minimum filter NTP uses. */
typedef struct atkdaq_sclock {
	double   period_ns;               /* 1e9 / fs */
	uint64_t n[ATKDAQ_SCLOCK_WIN];    /* sample index just after each transfer */
	int64_t  t[ATKDAQ_SCLOCK_WIN];    /* arrival, monotonic ns */
	int      head, count;
} atkdaq_sclock;

ATKDAQ_API void    atkdaq_sclock_init(atkdaq_sclock *c, double fs);
ATKDAQ_API void    atkdaq_sclock_add(atkdaq_sclock *c, uint64_t n_end, int64_t t_ns);
/* Lower-envelope intercept over the newest `last` entries (0 = all).
 * Returns 0 and writes *a_ns, or -1 when there are no entries. */
ATKDAQ_API int     atkdaq_sclock_intercept(const atkdaq_sclock *c, int last, double *a_ns);
/* Monotonic ns at which raw sample n was digitised (plus minimum latency). */
ATKDAQ_API int64_t atkdaq_sclock_time(const atkdaq_sclock *c, uint64_t n);

/* ------------------------------------------------------------------------ */
/* sync.c — integer delays from the noise source                            */
/* ------------------------------------------------------------------------ */

typedef struct atkdaq_sync_result {
	int32_t lag[ATKDAQ_MAX_CHANNELS];     /* x_i[j + lag[i]] ~ x_0[j]; lag[0] = 0 */
	float   ptn_db[ATKDAQ_MAX_CHANNELS];  /* peak-to-next, dB; ptn_db[0] = inf */
	float   frac[ATKDAQ_MAX_CHANNELS];    /* parabolic sub-sample part, diagnostic */
	float   worst_ptn_db;
} atkdaq_sync_result;

/* iq[i] points at n interleaved float I,Q samples of channel i, all taken
 * from the same raw index range. Searches lags in [-max_lag, +max_lag].
 * Returns 0, or -1 if n or max_lag is unusable, -2 on allocation failure. */
ATKDAQ_API int atkdaq_sync_estimate(const float *const *iq, int n_ch, int n, int max_lag,
                                    atkdaq_sync_result *out);

/* ------------------------------------------------------------------------ */
/* cal.c — weights and fractional delays from the noise source              */
/* ------------------------------------------------------------------------ */

typedef struct atkdaq_cal_result {
	float w_re[ATKDAQ_MAX_CHANNELS];
	float w_im[ATKDAQ_MAX_CHANNELS];
	float frac[ATKDAQ_MAX_CHANNELS];       /* samples, see atkdaq_frame.h */
	float coherence[ATKDAQ_MAX_CHANNELS];  /* band-mean |coherence| with channel 0 */
	float resid_deg[ATKDAQ_MAX_CHANNELS];  /* RMS residual phase after correction */
	float power_db[ATKDAQ_MAX_CHANNELS];   /* band power, dB re full scale */
	float spread_deg;                      /* max over channels of resid_deg */
	int   bins_used;
} atkdaq_cal_result;

/* Segment length for the Welch cross-spectra, and the half-width of the
 * band the fit uses, as a fraction of fs (RTL2832U's decimation filter rolls
 * off beyond it). */
#define ATKDAQ_CAL_NFFT       1024
#define ATKDAQ_CAL_BAND       0.35
#define ATKDAQ_CAL_MIN_COH    0.5f
/* After a sync every fractional residual is within half a sample of zero.
 * A calibration on the current delays that measures more than this on some
 * channel has found that the integer delays moved (daq.c then syncs). */
#define ATKDAQ_FRAC_WHOLE     0.75

/* iq[i]: n aligned interleaved float I,Q samples per channel. Returns 0,
 * -1 unusable input, -2 allocation, -3 the noise source was not seen (band
 * coherence below ATKDAQ_CAL_MIN_COH on some channel; the result still holds
 * the measured coherence so the message can name the channel). */
ATKDAQ_API int atkdaq_cal_estimate(const float *const *iq, int n_ch, int n, double fs,
                                   atkdaq_cal_result *out);

/* ------------------------------------------------------------------------ */
/* drops.c — has one channel lost samples the others did not?               */
/* ------------------------------------------------------------------------ */

/* A channel that loses D samples keeps receiving transfers at the same wall
 * times as the others, but each now ends D samples EARLIER in that channel's
 * own count — so its clock intercept (clock.c) jumps later by D sample
 * periods while the other channels' do not move. A host that is simply
 * slow moves every intercept together. So each channel's intercept shift
 * since the last sync is compared with the MEDIAN shift of all channels:
 * the median is the common mode (host latency, or a loss every channel
 * shared, which keeps the array aligned), and what stands out from it is a
 * channel that lost samples alone (which misaligns the array).
 *
 * This is a timing measurement and its floor is the arrival jitter of
 * Windows USB transfers — a few hundred microseconds, i.e. hundreds of
 * samples. A smaller loss is invisible here, which is why the policy also
 * re-measures the delays on the noise source periodically (sched.c) and why
 * atkdf watches a reference emitter's steering vector. */
typedef struct atkdaq_drops {
	int     n_ch;
	double  period_ns;
	int     recent;                         /* transfers in the detection window */
	double  ref_ns[ATKDAQ_MAX_CHANNELS];    /* each channel's intercept at the last rebase */
	int     have_ref;
	double  thresh_ns;                      /* smallest shift reported */
	int     persist;                        /* evaluations a shift must survive */
	int     streak[ATKDAQ_MAX_CHANNELS];
	int64_t est_samples[ATKDAQ_MAX_CHANNELS]; /* shift beyond the common mode, samples */
	int64_t common_samples;                 /* the common-mode shift, samples */
} atkdaq_drops;

ATKDAQ_API void atkdaq_drops_init(atkdaq_drops *d, int n_ch, double fs, int recent,
                                  double thresh_ns, int persist);
/* Record the current intercepts as the aligned state. Returns 0, or -1 if a
 * clock has no entries yet. */
ATKDAQ_API int  atkdaq_drops_rebase(atkdaq_drops *d, const atkdaq_sclock *const *clocks);
/* Re-evaluate. Returns a bit mask of channels whose own shift has stood out
 * from the common mode by more than the threshold for `persist` consecutive
 * evaluations; the size is in est_samples (positive = the channel lost
 * samples). */
ATKDAQ_API uint32_t atkdaq_drops_update(atkdaq_drops *d, const atkdaq_sclock *const *clocks);

#ifdef __cplusplus
}
#endif
#endif /* ATKDAQ_CORE_H */
