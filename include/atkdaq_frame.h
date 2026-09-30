/*
 * atkdaq_frame.h — THE WIRE FORMAT. Version 1.
 *
 * This header is the contract between atkdaq.exe (the producer) and every
 * consumer: ATK's kraken_source.py, atkdf's frame sources, and anything that
 * reads a .atkq file. python/atkdaq/frame.py implements the same layout and
 * tests/test_frames.py holds the two together. If this file and any other
 * document disagree about the format, this file wins.
 *
 * ---------------------------------------------------------------------------
 * THE STREAM
 * ---------------------------------------------------------------------------
 * A stream is a sequence of frames and nothing else — no preamble, no
 * trailer. A .atkq file is a stream written to disk verbatim; `atkdaq play`
 * replays one. On Windows stdout is switched to binary before the first byte
 * (text mode turns every 0x0A in the payload into 0x0D 0x0A, and nothing
 * downstream would ever explain why the array is misaligned). Status goes to
 * stderr as text; frames never do.
 *
 *     frame   = header (hdr_bytes) + payload (payload_bytes)
 *     header  = fixed part (ATKDAQ_FIXED_BYTES) + n_channels channel records
 *               at ch_offset, each ch_stride bytes
 *     payload = n_channels × samples_per_ch × 2 bytes, CHANNEL-MAJOR:
 *               ch0 I0 Q0 I1 Q1 … | ch1 I0 Q0 … | …    (fmt 0: int8)
 *
 * All multi-byte fields are little-endian. Fields are naturally aligned
 * within the header, so the struct below has no padding on any compiler this
 * project supports; the static asserts in frame.c check it.
 *
 * FORWARD COMPATIBILITY. A reader locates the channel records with ch_offset
 * and ch_stride and the payload with hdr_bytes — never with sizeof — so a
 * later version may APPEND fields to the fixed part or to the channel record
 * without breaking a v1 reader. Anything that changes the meaning of an
 * existing field is a new `version`, and a reader refuses a version it does
 * not know, saying which it wanted.
 *
 * INTEGRITY. hdr_crc32 is the CRC-32 (IEEE 802.3, the one zlib.crc32
 * computes) of the hdr_bytes header bytes with the crc field itself set to
 * zero. A reader that loses its place — a partial read, garbage on the pipe,
 * a truncated file — finds the next "ATKQ" whose CRC checks rather than
 * trusting four bytes of magic.
 *
 * ---------------------------------------------------------------------------
 * ALIGNMENT — WHAT "DELAYS ARE APPLIED; WEIGHTS ARE NOT" MEANS EXACTLY
 * ---------------------------------------------------------------------------
 * Let r_i[n] be the n-th raw sample received on channel i since the stream
 * started (n counts from 0 on each channel independently). Payload sample j
 * of channel i in a frame is
 *
 *     x_i[j] = r_i[ stream_sample0 + j + delay[i] ]          delay[0] == 0
 *
 * so the DAQ has already aligned the channels to the nearest sample: after
 * sync, x_0[j] … x_4[j] were digitised within one sample period of each
 * other. What is left is small and is REPORTED, not applied:
 *
 *   * frac_delay[i] (samples) — the residual delay of channel i behind
 *     channel 0 after integer alignment. Five RTL2832U decimators on one
 *     28.8 MHz clock start at arbitrary phases of that clock, so the
 *     residual is a fraction of a sample, and at 2.4 MSPS a 0.3-sample
 *     residual is a 22° phase error on a signal 500 kHz off centre. It
 *     cannot be corrected by an integer offset and it is not one number
 *     across the band: it is a phase SLOPE. (KrakenSDR's own Heimdall
 *     firmware measures the same thing and removes it by nudging each
 *     receiver's sample clock; atkdaq measures it and hands it over.)
 *   * w[i] = w_re + j·w_im — the complex gain that maps channel i onto
 *     channel 0 at the centre of the band. w[0] == 1.
 *
 * The model the DAQ fits on the noise source is
 *
 *     X_i(f) = X_0(f) · exp(−j·2π·f·frac_delay[i]/fs) / w[i]
 *
 * with f the baseband offset from fc in Hz, so a consumer calibrates with
 *
 *     Y_i(f) = w[i] · X_i(f) · exp(+j·2π·f·frac_delay[i]/fs)
 *
 * and, for a narrowband signal at baseband offset f0, that is the single
 * complex factor  c_i(f0) = w[i] · exp(+j·2π·f0·frac_delay[i]/fs).
 * python/atkdaq/frame.py `Frame.calibration_at(f0)` computes it, and atkdf's
 * calibration module applies it. Applying the weight without the slope is
 * the mistake this paragraph exists to prevent.
 *
 * The noise source calibrates the RECEIVERS only. Cables, connectors,
 * antennas and the roof are upstream of the switch and are not in these
 * numbers; that is atkdf's measured manifold.
 *
 * ---------------------------------------------------------------------------
 * TIME
 * ---------------------------------------------------------------------------
 * t_utc_ns is the host's estimate of the UTC instant channel 0's first
 * payload sample was digitised, from a clock model fitted to the USB
 * transfer arrival times (the lower envelope of arrival latency — see
 * src/clock.c), not the arrival time of whichever transfer happened to carry
 * it. stream_sample0 counts channel-0 raw samples since the stream started
 * and increases by samples_per_ch per frame INCLUDING frames that were
 * dropped on the pipe, so a consumer can compute exactly how much it
 * missed. seg_sample0 is the same count from the start of the current
 * segment.
 *
 * ---------------------------------------------------------------------------
 * RULES THE FLAGS ENCODE
 * ---------------------------------------------------------------------------
 *  * NOISE_ON — the payload is the calibration noise source, not the
 *    antennas, or is inside the settle window either side of a switch. A
 *    consumer that direction-finds a NOISE_ON frame gets a bearing of
 *    nothing.
 *  * MISALIGNED — the DAQ believes the channels are NOT sample-aligned:
 *    never synced, or a sample shortfall on one channel has been detected
 *    since the last sync. Never direction-find a MISALIGNED frame.
 *  * SEGMENT_START / segment — a segment is one tuning (fc, fs, gain). A
 *    retune or gain change starts a new one; a recording never straddles two
 *    segments pretending to be one capture.
 *  * passport — increments on every sync and every cal. Two frames with the
 *    same passport were aligned and calibrated by the same measurement; a
 *    consumer records the passport with every bearing.
 *  * DISCONTINUITY — this frame does not follow the previous one
 *    contiguously on every channel (frames dropped on the pipe, an overrun,
 *    or a new delay applied). Filters with memory reset here.
 *  * SYNTHETIC — produced by the synthetic test device, never by antennas.
 *    Set on every frame of a synthetic stream so a fixture can never be
 *    mistaken for a capture.
 */
#ifndef ATKDAQ_FRAME_H
#define ATKDAQ_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATKDAQ_MAGIC            "ATKQ"
#define ATKDAQ_VERSION          1
#define ATKDAQ_MAX_CHANNELS     16
#define ATKDAQ_FIXED_BYTES      96
#define ATKDAQ_CH_STRIDE        32

/* fmt */
#define ATKDAQ_FMT_CI8          0   /* int8 I,Q interleaved; value = raw_u8 - 128 */

/* flags (uint32) */
#define ATKDAQ_F_MISALIGNED     (1u << 0)
#define ATKDAQ_F_NOISE_ON       (1u << 1)
#define ATKDAQ_F_SEGMENT_START  (1u << 2)
#define ATKDAQ_F_CAL_STALE      (1u << 3)
#define ATKDAQ_F_OVERRUN        (1u << 4)   /* samples lost upstream inside this frame's span */
#define ATKDAQ_F_RETUNE         (1u << 5)   /* tuning or gain not yet settled */
#define ATKDAQ_F_CLIPPED        (1u << 6)   /* some channel's clip count >= the clip threshold */
#define ATKDAQ_F_DISCONTINUITY  (1u << 7)
#define ATKDAQ_F_SYNTHETIC      (1u << 8)

/* cal_state */
#define ATKDAQ_CAL_NONE         0
#define ATKDAQ_CAL_SYNCED       1   /* integer delays measured; weights not yet */
#define ATKDAQ_CAL_CALIBRATED   2
#define ATKDAQ_CAL_MISALIGNED   3
#define ATKDAQ_CAL_CALIBRATING  4   /* noise source on, measuring */

/* device_kind */
#define ATKDAQ_DEV_KRAKEN       1
#define ATKDAQ_DEV_SYNTH        2

/* mode */
#define ATKDAQ_MODE_STATIC      0
#define ATKDAQ_MODE_MOBILE      1

/* The fixed part: 96 bytes, every field naturally aligned. */
typedef struct atkdaq_hdr {
	char     magic[4];          /*  0 "ATKQ" */
	uint16_t version;           /*  4 ATKDAQ_VERSION */
	uint16_t hdr_bytes;         /*  6 whole header, fixed part + channel records */
	uint16_t ch_offset;         /*  8 byte offset of channel record 0 */
	uint16_t ch_stride;         /* 10 bytes per channel record */
	uint8_t  n_channels;        /* 12 5 for a KrakenSDR */
	uint8_t  fmt;               /* 13 ATKDAQ_FMT_* */
	uint8_t  cal_state;         /* 14 ATKDAQ_CAL_* */
	uint8_t  device_kind;       /* 15 ATKDAQ_DEV_* */
	uint32_t flags;             /* 16 ATKDAQ_F_* */
	uint32_t seq;               /* 20 +1 per ASSEMBLED frame; a gap = frames lost on the pipe */
	int64_t  t_utc_ns;          /* 24 UTC of channel 0's first payload sample */
	uint64_t fc_hz;             /* 32 centre frequency */
	uint32_t fs_hz;             /* 40 sample rate, per channel */
	uint32_t samples_per_ch;    /* 44 120000 at 2.4 MSPS / 50 ms */
	uint32_t segment;           /* 48 +1 on every retune / rate / gain change */
	uint32_t passport;          /* 52 +1 on every sync or cal */
	uint32_t cal_age_ms;        /* 56 time since the passport's measurement */
	uint16_t spread_cdeg;       /* 60 post-cal residual phase spread, centi-degrees (0xFFFF = none) */
	uint16_t sync_conf;         /* 62 0..65535 sync confidence (peak-to-next, see sync.c) */
	int16_t  gain_tenths_db;    /* 64 identical on all channels by construction */
	uint16_t mode;              /* 66 ATKDAQ_MODE_* */
	uint32_t payload_bytes;     /* 68 n_channels * samples_per_ch * 2 */
	uint64_t stream_sample0;    /* 72 channel-0 raw index of this frame's first sample */
	uint64_t seg_sample0;       /* 80 the same, counted from the segment's start */
	uint32_t hdr_crc32;         /* 88 CRC-32 of the header with this field zero */
	uint32_t reserved;          /* 92 zero */
} atkdaq_hdr;

/* One per channel, ch_stride bytes, starting at ch_offset. */
typedef struct atkdaq_ch {
	int32_t  delay;             /*  0 integer samples APPLIED (see ALIGNMENT); 0 on channel 0 */
	float    frac_delay;        /*  4 residual delay behind channel 0, samples, NOT applied */
	float    w_re;              /*  8 calibration weight, NOT applied; 1 on channel 0 */
	float    w_im;              /* 12 */
	uint32_t drops;             /* 16 samples believed lost on this channel since the last sync */
	uint32_t clip;              /* 20 samples in THIS frame with I or Q at the ADC rail */
	uint64_t sample_count;      /* 24 raw samples received on this channel since stream start */
} atkdaq_ch;

#define ATKDAQ_HDR_BYTES(n)     (ATKDAQ_FIXED_BYTES + (n) * ATKDAQ_CH_STRIDE)
#define ATKDAQ_PAYLOAD_BYTES(n, s) ((uint32_t)(n) * (uint32_t)(s) * 2u)

#ifdef __cplusplus
}
#endif
#endif /* ATKDAQ_FRAME_H */
