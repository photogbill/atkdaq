# atkdaq — Coherent multi-channel acquisition: Build Plan (v1)

> **Status (2026-09-30):** Phases 0–4 built and tested end to end against a
> synthetic coherent receiver (§11.9); everything that needs the KrakenSDR
> itself is listed in §11.13 and answered by T1/T2 (§8). KrakenSDR ordered,
> not in hand. §0–§10 are the plan as written before the build; where the
> build changed it, the text says so and **§11 records what was built and
> why**. Where §2 and `include/atkdaq_frame.h` differ, the header file wins
> (it always did). This is the plan for the **DAQ program**
> only. The DF engine is `atkdf` (its own repo); the ATK-side wiring is in
> ATK's `docs/KRAKEN_DF_PLAN.md`, which is the umbrella. Where the three
> disagree about the wire format or the DAQ's behaviour, **this document
> wins** — the frame header in `include/atkdaq_frame.h` is the contract the
> other two program against.
>
> **What it is.** One console program, `atkdaq.exe`, that owns a coherent
> multi-channel receiver (first and only device: the KrakenSDR — five R820T2 +
> RTL2832U chains on one 28.8 MHz clock with a switched noise source), and
> emits framed, sample-aligned, N-channel blocks with their calibration state
> on a pipe. It knows nothing about antennas, geometry, bearings or maps.
>
> **Why its own program, and why GPL.** It links `librtlsdr` — the
> krakenrf fork, which adds the two calls the Kraken needs (`rtlsdr_set_dithering`,
> the GPIO calls for the noise-source switch) — and librtlsdr is GPL-2.0. ATK
> is all-rights-reserved, so the DAQ ships as its own GPL-2.0 program exactly
> as `rtl_sdr.exe` and dsd-neo do in ATK's `bin\`. It also means a USB fault
> or a driver crash cannot take ATK down, and it makes the DAQ testable with a
> file (§7) and no GUI.

---

## 0. Conventions (from atkdsp)

C99, CMake, MSVC (the build tools ATK's `install.bat` already requires),
`build.bat` that finds `vcvars64.bat` itself and drives `cl.exe` if CMake is
absent, `bin\` for outputs, one header that **is** the ABI, a `python/`
package with a numpy twin of every algorithm the C implements and the
cross-check tests that hold the two together. `NOT /fp:fast`. Nothing
allocates in the streaming path after start.

## 1. Layout

```
atkdaq/
  include/atkdaq_frame.h     THE WIRE FORMAT. Read its header comment first.
  src/
    main.c                   args, config, lifecycle, exit codes
    config.c                 config file (serial→channel map, rate, gain, fc)
    devices/kraken.c         open by serial, configure, dither off, AGC off, noise-source GPIO
    devices/device.h         the device vtable: open/close/tune/gain/noise/start/stop
    readers.c                one async read thread per channel → per-channel ring
    ring.c                   power-of-two byte ring, single producer / single consumer
    sync.c                   noise-source cross-correlation → integer delays (+confidence)
    cal.c                    noise-source phase/amplitude weights (+post-cal spread)
    drops.c                  per-channel sample accounting; inter-channel divergence; flags
    frame.c                  block assembly (apply delays by read-pointer offset) + framing
    control.c                stdin command parser; stderr status lines
    sched.c                  the calibration policy state machine (§5)
    clock.c                  QPC → UTC, monotonic sample counters
    log.c
  python/atkdaq/
    __init__.py              open_stream(argv) → iterator of Frame; Control (stdin client)
    frame.py                 parse/serialise the header; numpy views onto payload
    reference.py             numpy twins: xcorr sync, cal weights, spread — cross-checked vs C
    play.py                  replay a .atkq frame file as if live (the device-less test path)
  tools/
    atkdaq_record            dump raw frames to a .atkq file (fixtures)
    atkdaq_play              replay a .atkq file on stdout at wall-clock rate
    atkdaq_probe             enumerate, print serials, test throughput, run one sync+cal, report
  vendor/
    librtlsdr/               krakenrf fork, PINNED commit, built as a static lib into atkdaq.exe
    libusb/                  prebuilt libusb-1.0 (LGPL, dynamic) — DLL beside atkdaq.exe
    pocketfft/               (BSD-3) for the sync cross-correlation
  tests/
    test_smoke.c             frame round-trip; ring; drops accounting on synthetic counters
    test_frames.py           parse the fixture file; header invariants
    test_cross_check.py      sync/cal twins vs the C on synthetic multi-channel data
    fixtures/                small .atkq files from T1/T2 (§8), committed
  bench/bench_throughput.py
  build.bat · CMakeLists.txt · README.md · LICENSE (GPL-2.0) · docs/ATKDAQ_PLAN.md
```

## 2. The wire format — `include/atkdaq_frame.h`

> **Built differently — see §11.1.** The struct below is the planning draft.
> The built format adds a CRC, located channel records, fractional delays,
> sample-index time and three flags. Program against the header file.

Binary frames on stdout (Windows: `_setmode(_fileno(stdout), _O_BINARY)`
before the first byte, or every `0x0A` in the payload becomes `0x0D 0x0A`
and nothing downstream will ever explain why the array is misaligned).
A named pipe is the alternative transport, same frames. Status lines go to
stderr as `key=value` text; frames never do.

```
struct atkdaq_hdr {                    // little-endian, packed, 4-byte aligned fields
  char     magic[4];        // "ATKQ"
  uint16_t version;         // 1
  uint16_t hdr_bytes;       // sizeof this header incl. per-channel arrays
  uint8_t  n_channels;      // 5 for a Kraken; the format is device-agnostic
  uint8_t  fmt;             // 0 = int8 I,Q interleaved
  uint16_t flags;           // bit0 misaligned, bit1 noise_on (payload is noise!), bit2 segment_start,
                            // bit3 cal_stale, bit4 overrun_this_block, bit5 retune_in_progress
  uint32_t seq;             // monotonically increasing; a gap = frames lost on the pipe
  int64_t  t_utc_ns;        // host UTC of the first sample of channel 0 in this block
  uint64_t fc_hz;
  uint32_t fs_hz;           // per channel
  int16_t  gain_tenths_db;  // identical on all channels by construction
  uint8_t  cal_state;       // 0 none, 1 synced, 2 calibrated, 3 misaligned, 4 calibrating
  uint8_t  device_kind;     // 1 = KrakenSDR
  uint32_t samples_per_ch;  // 120000 at 2.4 MSPS / 50 ms
  uint32_t segment;         // increments on every retune / rate / gain change
  uint32_t passport;        // increments on every sync or cal; identifies the weights below
  uint32_t cal_age_ms;
  uint16_t spread_cdeg;     // post-cal phase spread across channels, centi-degrees, on noise
  uint16_t sync_conf;       // 0..65535, peak-to-next ratio of the sync correlation
  // per-channel arrays, n_channels each:
  int32_t  delay[N];        // integer sample delay applied to channel i (relative to ch 0)
  float    w_re[N], w_im[N];// calibration weight for channel i (NOT applied to the payload)
  uint32_t drops[N];        // samples lost on channel i since the last sync
  uint64_t sample_count[N]; // running count since start (after delay application)
};
// payload: n_channels × samples_per_ch × 2 bytes, channel-major, int8 I,Q
```

Rules the header encodes:

- **Delays are applied; weights are not.** The DAQ aligns the channels in
  time (a read-pointer offset per channel, costing nothing) and *reports*
  the phase/amplitude weights. The consumer applies them in float, which
  keeps the wire int8, keeps the DAQ dumb, and lets a measured antenna
  manifold be applied on top without a round trip.
- **`noise_on` means the payload is the noise source, not the antennas.**
  A consumer that DFs a noise-on frame gets a bearing of nothing. The flag
  is set on every frame captured with the switch thrown, including the
  settle frames either side.
- **`segment` starts over on any change of tuning.** A clip never straddles
  two tunings pretending to be one capture — the same rule as ATK's §RF-F9
  ring.
- **`passport` identifies the calibration.** Two frames with the same
  passport were calibrated by the same measurement; a consumer records the
  passport with every bearing.
- The header is versioned; a consumer refuses a version it does not know
  and says which it wanted.

## 3. Devices — `devices/kraken.c`

- Enumerate all RTL devices; select by the **five serials** in the config
  (`channel[i].serial`); refuse to start with any missing and name it.
  Serial order *is* channel order; the antenna→connector map is atkdf's
  business, not this program's.
- Per device: **dithering off FIRST** (`rtlsdr_set_dithering(dev, 0)`),
  then tuner gain mode manual, centre frequency, **tuner gain identical**,
  sample rate (verified exact), AGC off, bias tees, reset buffer. *(Corrected
  in the build: the order first written here put dithering last, which
  leaves it on — see §11.2.)* The fork is compiled in statically, so stock
  librtlsdr cannot be linked by mistake.
- Noise source: the GPIO on the channel-0 device per the fork's Heimdall
  usage; `noise(on)` sets it and records the host time; frames within the
  settle window either side carry `noise_on`.
- Retune: `rtlsdr_set_center_freq` on all five *without stopping the
  streams*; `segment++`; `retune_in_progress` set until the tuners report
  the new frequency and a settle window has passed; then the policy (§5)
  decides sync vs cal-only.
- The vtable in `device.h` is what a second coherent device would implement.
  Do not generalise further than that until one exists.

## 4. Readers, ring, framing

- `rtlsdr_read_async` per device with a 256 KB transfer size, callback
  copies into that channel's ring and adds to `sample_count`, stamping the
  host QPC time on the first byte of each transfer into a small side ring
  (for the block's `t_utc_ns` and for `drops.c`).
- Rings: power-of-two, 4 s deep at 2.4 MSPS (19.2 MB per channel), single
  producer / single consumer, no locks, no allocation.
- Frame assembler: waits until every channel holds `samples_per_ch` beyond
  its (delay-offset) read pointer, builds the header, writes header +
  payload with one `WriteFile` per frame, advances all read pointers. Writes
  are never allowed to block the readers: if the consumer stalls, the
  assembler drops whole frames, counts them in a stderr status line, and
  keeps the rings from wrapping over unread data by advancing all channels
  together — so a stall costs frames, never alignment.

## 5. Sync, calibration, drops, policy

**Sync** (`sync.c`). Noise on; discard the settle window; take 200 ms from
all channels; cross-correlate channels 1..N−1 against 0 (FFT via pocketfft,
magnitude, peak); the lag is the integer delay; `sync_conf` is peak-to-next;
take a second 200 ms and require the same lag to the sample; noise off.
Apply. Blind ≈ 0.5 s.

**Calibration** (`cal.c`). Noise on; 50–100 ms of *aligned* samples; the
cross-spectrum of each channel against channel 0 averaged over the noise's
flat band gives the complex gain; weights are its conjugate normalised to
channel 0; `spread_cdeg` is the residual phase spread across channels after
applying them (on the same noise data); noise off. Blind ≈ 0.1 s.
`passport++`.

**Drops** (`drops.c`). Per channel: bytes received vs bytes expected from
elapsed host time × fs, and the divergence between channels' `sample_count`.
Any inter-channel divergence of ≥ 1 sample sets `misaligned` and
`cal_state=3`. The count of every shortfall goes in `drops[i]`.

**Policy** (`sched.c`), driven by the consumer's chosen mode (`mode static|mobile`):

| event | static | mobile |
|---|---|---|
| start | sync + cal | sync + cal |
| retune | cal only if the §6 hypothesis held, else sync + cal | same |
| gain change | cal | cal |
| `misaligned` | sync at the next quiet moment (consumer may say `quiet`) | sync immediately |
| periodic `sync_check` | every 300 s, 100 ms of noise, change delays only if two checks agree | every 60 s, same rule |
| consumer `cal` / `sync` | on demand | on demand |

The policy is in the DAQ so that a consumer that has crashed still leaves a
coherent DAQ behind; the consumer can override every interval.

## 6. The fast-retune hypothesis — settle it on day one

Integer delays come from the five USB streams starting at different
instants. The ADC clock and the streams never stop on a retune, so the
delays *should* survive one; only the tuner PLL phases (the weights) should
change. If true, a retune costs a tuner settle plus 0.1 s of cal, and a
**dwell schedule** across several channels becomes practical (ATK plan
§4.6). If false, a retune costs 0.6 s and the schedule is merely slower.
`atkdaq_probe --retune-test` measures it: sync, record delays, retune ±1 MHz
ten times, re-sync each time, print whether the delays moved. This runs in
T1 before anything is built on the answer.

## 7. Testing without the hardware

- `atkdaq_record` writes raw frames to a `.atkq` file (header + payload,
  verbatim). `atkdaq_play` replays one on stdout at wall-clock rate with
  the same header semantics. Every consumer test runs against a file.
- `python/atkdaq/reference.py` reimplements sync and cal in numpy;
  `test_cross_check.py` plants known delays and gains on synthetic
  five-channel noise and requires the C and the twin to agree to the sample
  and to 0.1°.
- `test_frames.py` parses the committed fixtures and checks every invariant
  in §2.
- The first real fixtures come from T1 (noise source) and T2 (splitter):
  ten seconds each, committed small.

## 8. Acceptance (T1, T2 — the umbrella plan's §12)

- **T1 noise source, 10 min:** `spread_cdeg` < 200 (2°) and flat; drops
  zero on a dedicated root port; a forced disturbance (plug a different
  device into the same hub) is *detected* (`misaligned` set) and resynced
  within the policy's time; `--retune-test` answers §6.
- **T2 splitter:** one antenna → 5-way splitter → five equal cables →
  inputs: spread < 2° regardless of what is on the air, for ten minutes.
- **Throughput:** 24 MB/s sustained to a consumer that reads with a 4 MB
  pipe buffer; a stalled consumer costs frames, never alignment.
- `atkdaq_probe` prints a one-screen report an operator can read: serials
  found, throughput, sync delays and confidence, spread, and a plain
  verdict.

## 9. Phases

| phase | builds | done when | status 2026-09-30 |
|---|---|---|---|
| **0** (no hardware) | vendor the fork pinned; build it and libusb on MSVC; `device.h`; `frame.h` + `frame.c`; `ring.c`; `python/atkdaq/frame.py`; `atkdaq_play`; synthetic `.atkq` fixture generator; `reference.py` sync/cal twins | a synthetic fixture round-trips C → pipe → Python; twins pass on planted delays/gains | **done** (MSVC build untested here; MinGW + wine pass, §11.10) |
| **1** | `kraken.c`; readers; assembler; `atkdaq_probe` enumerate/throughput; `atkdaq_record` | five channels stream to a file on the real unit; throughput report | **built**; needs the unit |
| **2** | `sync.c`, `cal.c`, `drops.c`, `control.c`, `sched.c` static mode | **T1 passes**; §6 answered | **built, passes on the synthetic device**; T1 needs the unit |
| **3** | mobile policy; `sync_check`; `quiet`; retune-without-stop; stall handling proven; docs | **T2 passes**; a consumer stall drops frames and keeps alignment | **built**; stall proven on the synthetic device; T2 needs the unit |
| **4** | polish: exit codes, config validation messages, `README` operator notes (Zadig per interface, dedicated root port, 5 V ≥ 2.4 A) | ATK's `get_atkdaq.bat` builds it on a clean machine | **built**; README in words (screenshots when the unit is here) |

## 10. Risks and limits

- **Zadig/WinUSB** must be applied to *each* of the five RTL interfaces;
  document the exact procedure with screenshots in the README — it is the
  first thing that will go wrong on a new laptop.
- **The fork's build on MSVC.** osmocom librtlsdr builds on Windows with
  libusb; the krakenrf fork is a small delta on it. If its CMake resists,
  compile the handful of `.c` files directly in `build.bat` as atkdsp does
  without CMake.
- **USB on Windows is not real-time.** Laptops throttle; hubs share. The
  program's job is to detect, report and resync, and to say "dedicated root
  port" in the probe's report when drops appear.
- **libusb event handling** runs on its own thread per context; never call
  librtlsdr from the callback. *(Built with one context per device, which is
  what librtlsdr does — §11.3.)*
- **Power.** 5 V ≥ 2.4 A on the Kraken's power port; a laptop port browns
  it out under load and the symptom is drops on one channel, not an error.
- **Blind calibration** is time not listening; the policy trades one for
  the other and the consumer can override it.
- **Not in scope:** applying weights, antenna manifolds, DoA, geometry,
  GPS, anything with a window. All of that is `atkdf`.

---

## 11. Build notes (2026-09-30)

What was built, where it differs from §0–§10, and why. Each note names the
file that holds the detail.

### 11.1 The wire format grew (`include/atkdaq_frame.h`)

- **Located, extensible layout.** The header is a 96-byte fixed part plus one
  32-byte record per channel. A reader finds the records with `ch_offset` and
  `ch_stride` and the payload with `hdr_bytes`, never with `sizeof`. A later
  version can append fields without breaking a v1 reader. A change of meaning
  is a new `version`, and readers refuse a version they do not know, saying
  which one they wanted.
- **`hdr_crc32`** is zlib's CRC-32 of the header, computed with the field
  itself zeroed. A reader that loses its place skips forward to the next
  `ATKQ` whose CRC checks. The CRC is checked **before** the version, so a
  damaged version byte reads as damage, never as "a newer format".
- **`frac_delay` per channel.** This is the plan's biggest omission. The five
  RTL2832U decimators start at arbitrary phases of the shared clock, so after
  integer alignment each channel still trails channel 0 by a fraction of a
  sample. At 2.4 MSPS, 0.3 samples is 22° on a signal 500 kHz off centre.
  That is a phase **slope** across the band, so a single complex weight is
  right only at band centre. The DAQ measures the slope and reports it; it
  does not apply it. The consumer's correction at baseband offset f0 is
  `w · exp(+j2π·f0·frac/fs)`. KrakenSDR's Heimdall removes the same error by
  nudging each receiver's sample rate; atkdaq hands it over instead.
- **Time is a sample index.**
  - `stream_sample0` is channel 0's raw index of the frame's first sample. It
    advances by `samples_per_ch` per frame, including frames dropped on the
    pipe, so a consumer knows exactly what it missed.
  - `seg_sample0` is the same index counted from the start of the segment.
  - `t_utc_ns` comes from a lower-envelope fit of the transfer arrival times
    (`clock.c`), not from whichever transfer carried the sample.
- **Other additions:**
  - `flags` widened to 32 bits, with three new flags: `CLIPPED`,
    `DISCONTINUITY` (filters with memory reset here) and `SYNTHETIC` (set on
    every frame from the synthetic device, so a fixture is never mistaken for
    a capture).
  - `mode` is now in the header.
  - `clip` gives a per-channel rail count for each frame.
  - `sync_conf` is the worst channel's peak-to-next ratio in centi-dB.
  - `spread_cdeg` = 0xFFFF means "not calibrated".
  - `sample_count` counts **raw** samples received, not samples after the
    delay is applied, so a loss shows directly.

### 11.2 Device bring-up order (`src/devices/kraken.c`; corrects §3)

**Dithering goes off before the first `set_center_freq`.** In the fork,
`rtlsdr_set_dithering` only records the setting. The R820T register is
written when the PLL is programmed, which happens inside `set_center_freq`
(`r82xx_set_pll`). With the order first written in §3 (dithering last), the
setting would not take effect until the first retune.

The order as built, per device:

1. Open by serial. If one is missing, the error names it and lists the
   serials that are present.
2. Check the tuner is an R820T or R828D.
3. Dithering off.
4. Manual gain mode.
5. Set fc.
6. Set gain.
7. Set fs, and read it back to verify it is exact.
8. AGC off.

Then, on channel 0's chip, the noise GPIO goes off and the bias tees are set.
Each device gets `reset_buffer` just before its reader starts.

- **Noise source:** `rtlsdr_set_bias_tee_gpio(dev_ch0, noise_gpio, on)`, with
  `noise_gpio` = 0 by default.
- **Bias tees:** channel m's is GPIO m+1 on channel 0's chip.

Both mappings follow Heimdall's usage. **They are unverified until the
hardware arrives** (§11.13).

### 11.3 One libusb context per device (differs from §10)

`rtlsdr_open` creates its own libusb context for each device. "One context,
five handles" would mean modifying the fork, so the build runs five contexts,
each with its own reader thread inside `rtlsdr_read_async`. The readers start
together behind a barrier. The callback only copies bytes into the channel's
ring and records one monotonic-clock stamp per transfer; nothing calls
librtlsdr from it.

### 11.4 One program, subcommands (differs from §1 `tools/`)

`atkdaq run | record | play | probe | enum | help-config | version` replaces
three separate tool programs. That means one binary to build, ship and find,
and the tools share the engine. The readers and the assembler live in
`daq.c`, the engine, rather than in separate `readers.c` and assembler files.
`frame.c` holds the header, CRC and conversion.

### 11.5 Engine rules (`src/daq.c`, `src/ring.c`)

- **The rings never block the producer.** They are single-producer
  single-consumer and use absolute positions. A read that the producer has
  overtaken is refused, never returned torn.
- **A stall costs frames, never alignment.** Frames go to a writer thread
  through a fixed pool.
  - If the pool is empty the frame is dropped: `seq` still advances and the
    next frame carries `DISCONTINUITY`.
  - If a ring is about to overwrite unread data, every channel skips to the
    same aligned position together (`OVERRUN`).
- **Every decision is made by sample index, never by wall clock.** Noise-on
  intervals, retune settle, segment boundaries and capture windows are all
  indexes. That is why the synthetic device can run at any speed and behave
  exactly as it does in real time.

### 11.6 Sync and calibration as built (`src/sync.c`, `src/cal.c`; §5)

- **Sync** takes two back-to-back captures of `sync_samples` (131072, i.e.
  55 ms at 2.4 MSPS) and searches lags of ±`sync_max_lag` (32768) around a
  coarse prior.
  - The prior is the current delays if synced, the delays corrected by the
    measured loss if misaligned, or the arrival-time clocks if never synced.
  - The two captures must agree to the sample.
  - The worst peak-to-next ratio must be at least 10 dB. The "next" peak
    excludes ±16 lags around the main one, because the band-limited noise's
    sinc sidelobes otherwise cap the ratio.
- **Calibration** runs on the aligned tail of the same capture.
  - It averages Welch cross-spectra (1024 points, periodic Hann window, 50%
    overlap) over |f| ≤ 0.35·fs, excluding the 5 bins at DC, where the LO
    spike is correlated across channels.
  - The slope comes from products of neighbouring bins, so the phase is never
    unwrapped. The intercept follows, then a weighted least-squares refinement
    with weights coh²/(1−coh²).
  - The residual RMS is the spread. A calibration is refused if the spread is
    over 5° or if coherence is under 0.5 ("the noise source was not seen").
- **Periodic check:** the plan's "change delays only if two checks agree" is
  built as the two captures inside one check.
- **Time without antennas:**

  | procedure | settle | capture | settle after | total |
  |---|---|---|---|---|
  | sync + cal | 40 ms | 191 ms | 40 ms | ≈ 0.27 s |
  | cal only | 40 ms | 55 ms | 40 ms | ≈ 0.14 s |

  A retune adds 50 ms of retune settle. The plan estimated 0.5 s and 0.1 s.
- **Known answers** (synthetic device, 2.4 MSPS): integer delays exact; frac
  to about 1e-4 samples; gain to about 0.003 dB; phase to about 0.01°; spread
  about 0.16°. The C and the numpy twins in `reference.py` agree
  (`tests/test_cross_check.py`).
- **A cal that finds the delays moved syncs instead** (`ATKDAQ_FRAC_WHOLE`
  = 0.75 samples). After a sync every residual is within half a sample.
  - A cal-only procedure that measures more than 0.75 has found that the
    integer delays moved underneath it: a cal-only retune that did not keep
    them (§6), or a loss too small for `drops.c`.
  - Applied, those weights would be right at one frequency (the slope absorbs
    the shift) while the frames were a sample or more out of alignment, with
    nothing in the header to say so.
  - So the cal is not applied. The event `delays_moved` is logged and a sync
    runs at once. After a cal-only retune this also refutes the §6
    hypothesis.

### 11.7 Drop detection: what it can and cannot see (`src/drops.c`; refines §5)

The plan's "any inter-channel divergence of ≥ 1 sample" is not observable
from USB. Byte counts can only be compared at transfer granularity (131072
samples), and the arrival times jitter.

What is built: a channel that loses D samples keeps receiving transfers at
the same wall times, but its clock intercept jumps later by D/fs. Each
channel's shift since the last sync is compared with the **median** shift
across channels.

- The median is the common mode. The host being late moves every channel
  together; that is logged as `common_loss` and the array stays aligned.
- A channel that stands out from the median lost samples on its own.

The floor is the arrival jitter after the lower-envelope filter: hundreds of
microseconds, so hundreds of samples. The default threshold is 1 ms (2400
samples), and the shift must persist for 2 evaluations 100 ms apart. Smaller
losses are caught by the periodic check (every 300 s static, 60 s mobile),
which measures the delays directly on the noise source, and by the §11.6
guard.

**Detection comes after the fact.** The frames sent between the loss and its
detection went out unflagged. The `misaligned` event carries `since`, the
channel-0 index where the loss happened (recovered from the clock history),
and `from_s0`, the first frame flagged. A consumer that wants to be strict
does one of two things:

- discards what it derived from frames at or after `since`; or
- holds frames back for the detection latency (about 0.3 s) before using
  them.

### 11.8 Policy as built (`src/sched.c`; §5, §6)

- **The §5 table holds, with a bound on waiting.** In static mode a
  misaligned array waits for the consumer's `quiet` **or 5 s, whichever comes
  first**. Misaligned data is useless, so waiting longer only prolongs the
  uselessness.
- **§6 is learned at run time, not settled once in T1.** `retune_policy` = 0
  is the automatic mode:
  - The first three retunes each get sync + cal.
  - If all three kept the delays, later retunes get a cal only.
  - A retune whose sync (or, per §11.6, whose cal) shows moved delays refutes
    the hypothesis for the session. From then on every retune gets
    sync + cal.
  - `retune_policy` = 1 forces sync + cal on every retune; 2 forces cal only.
  - `probe --retune-test` still measures it deliberately: ten retunes of
    ±1 MHz, each with a forced sync.
- **Timing:** periodic timers start only after the first calibration. A
  failed procedure backs off for 2, 4, 8, 16, then 32 s; an explicit request
  overrides the back-off. Commands that arrive during a procedure are carried
  out when it ends.

### 11.9 The synthetic device (`src/devices/synth.c`; new — §7 had file replay only)

Replaying a file only replays what the program already wrote. To prove sync,
cal, drop detection and the policy end to end without hardware, the program
needs a device you can plant the truth into.

`--device synth` is a coherent receiver behind the same vtable as the Kraken:

- It builds signals in the frequency domain, one block at a time, so the
  planted gains, phases and fractional delays are exact per bin.
- It has a switched common noise source, emitters with per-element steering
  phases, a receiver noise floor and 8-bit quantisation.
- It can inject per-channel stream offsets, USB arrival jitter and sample
  loss at a planted time.
- A retune re-randomises every tuner phase. With
  `synth.retune_moves_delays=1` it also shifts the offsets by 0–2 samples.

It runs in real time or, with `synth.fast=1`, as fast as the consumer reads.
`tests/fixtures/synth_240k.atkq` was recorded from it.

### 11.10 Windows (`build.bat`, `vendor/win32/pthread.h`, `tests/check_windows_build.sh`)

- **The build.** `build.bat` drives `cl.exe` directly, with no CMake needed.
  It builds:
  1. libusb 1.0.29 from its pinned source, using `get_hackrf.bat`'s source
     list, as its own DLL (LGPL);
  2. librtlsdr's objects;
  3. `atkdaq.exe`;
  4. `atkdaq_core.dll`;
  5. the smoke test.
  `/cmake` is optional.
- **The pthread shim.** librtlsdr uses pthreads. The shim maps them onto
  `CRITICAL_SECTION` (recursive, which librtlsdr's locking requires),
  `CONDITION_VARIABLE` and `_beginthreadex`.
- **Commands on stdin** are read with raw `read()` / `ReadFile`, not stdio. A
  stdio read blocked in the command thread holds stdin's lock, and `exit()`
  then deadlocks trying to flush it.
- **Verification.** There is no MSVC in the build sessions. The Windows code
  paths were checked with a MinGW-w64 cross-build, and the whole test suite
  was run against that `.exe` under wine. **MSVC itself is untested:**
  `build.bat` on the real machine is the final word.

### 11.11 Licensing (affects the umbrella plan)

atkdaq is GPL-2.0, and so is its Python package (`python/atkdaq`). **ATK and
atkdf must not import it.** Importing a GPL module into your own process is,
on the conservative reading (the FSF's), creating a combined work. I'm not a
lawyer, and this arrangement avoids having to find out.

So the umbrella plan's "`kraken_source` wraps atkdaq's Python client" changes:

- ATK's `kraken_source` and atkdf's frame source both start `atkdaq.exe` as a
  separate process.
- Both read its stream with `atkdf.atkq`, atkdf's own reader, written from
  `include/atkdaq_frame.h`. That is the same arm's-length arrangement ATK
  already has with `rtl_sdr.exe` and dsd-neo.
- atkdf's tests hold its reader to this program's writer by reading a copy of
  `synth_240k.atkq`.

### 11.12 Numbers (on the build container, synthetic device)

- **Throughput:**
  - Real time, all five channels: 23.6 MB/s (0.99× the 24 MB/s the Kraken
    produces) with zero frames lost.
  - Fast mode: 34 MB/s, bound by the synthetic generator, not the pipeline.
- **Tests:**
  - C smoke test: 83 checks.
  - Python: 30 tests (frames, cross-checks, 15 end-to-end).
  - All pass on Linux, and against the Windows `.exe` under wine.

### 11.13 Open until the hardware arrives

- **The GPIO numbering on the real board** (§11.2). The first `probe` run
  shows it: "noise seen: coherence ≈ 1" means the switch works.
- **Real settle times:** tuner lock after a retune, and the noise switch's
  transient (`retune_settle_ms` 50, `settle_ms` 40).
- **USB timing on Windows,** and therefore the real drop-detection floor
  (§11.7).
- **§6:** whether a retune keeps the delays. The program learns this either
  way.
- **T1 and T2 (§8),** their fixtures, and the Zadig screenshots for the
  README.

### 11.14 Independent review (2026-09-30)

After the build, a separate reviewer agent (which had not written the code)
went over kraken.c, the wire format across all three languages, the GPL
boundary, and the DSP. It confirmed the wire format agrees byte-for-byte
(C `frame.c`, `python/atkdaq/frame.py`, and atkdf's independent `atkq.py`),
that the CRC-before-version order and the ch_offset/ch_stride location rules
hold, and that the coherence-critical setup (dither-before-tune, gain/AGC off,
per-device contexts, callback never calling librtlsdr) is correct. It found and
these were fixed in `kraken.c`:

- **Reader-thread shutdown interlock.** A reader passed the start barrier and
  entered `rtlsdr_read_async` unconditionally, so a failed partial start left
  orphan readers streaming into devices the caller then closed (use-after-free),
  and an immediate abort could cancel before the read was RUNNING and then hang
  in the join. Fixed: the reader returns if `running` cleared at the barrier;
  `kraken_start` unwinds through `kraken_stop` on a failed launch; `kraken_stop`
  retries `rtlsdr_cancel_async` until it arms.
- **R828D dithering.** `configure_one` accepted R828D, but the fork's
  `rtlsdr_set_dithering` only handles the R820T and would fail it with a
  misleading message. R828D is no longer accepted (a Kraken's R820T2 enumerates
  as R820T).
- **Bias-tee GPIOs** are now driven only when `bias_tee` is set, and a note
  records that GPIO 4 on chip 0 is documented as tuner-reset on a stock
  RTL2832U — one more thing for the first `probe` to confirm (§11.13).

The full test suite (C smoke 83, Python 30) and the MinGW+wine Windows build
stayed green through the fixes.
