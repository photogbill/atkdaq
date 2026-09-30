# atkdaq

Coherent multi-channel acquisition for the Analyst Toolkit. One console
program, `atkdaq.exe`, owns a coherent multi-channel receiver (first device:
the **KrakenSDR**, five R820T + RTL2832U chains on one 28.8 MHz clock with a
switched noise source). It emits framed, sample-aligned, N-channel blocks
on stdout, with their calibration state in every header. It knows nothing
about antennas, geometry or bearings; that is [atkdf](../atkdf).

**Status (2026-09-30):** built and tested end to end against a **synthetic
coherent receiver**. The KrakenSDR has not arrived yet. The Kraken code is
written against the krakenrf librtlsdr fork's real API and compiles and
links on Linux and for Windows. What only the hardware can answer is listed
under *Untested until the Kraken arrives*. The plan is `docs/ATKDAQ_PLAN.md`,
and its build notes (§11) record every decision made while building.

**Licence:** GPL-2.0 (`LICENSE`). atkdaq links the krakenrf fork of
`librtlsdr`, which is GPL. That is why it is a separate program that ATK talks
to over a pipe, just as ATK already does with `rtl_sdr.exe` and dsd-neo.
libusb is LGPL-2.1 and is built as its own DLL beside the exe. The
`python/atkdaq` package is part of atkdaq and is GPL too. **ATK and atkdf
do not import it.** They read the documented wire format with their own
reader, `atkdf.atkq`.

---

## Build

Windows (the target): run `build.bat`. It finds the MSVC build tools (the same
ones ATK's `install.bat` requires) and builds everything with `cl.exe`
directly, so CMake is not needed:

```
bin\atkdaq.exe          the program
bin\libusb-1.0.dll      libusb 1.0.29 (must sit beside atkdaq.exe)
bin\atkdaq_core.dll     the computing core, for the cross-check tests
bin\atkdaq_smoke.exe    the C smoke test (build.bat runs it)
```

`build.bat /cmake` uses CMake instead. On Linux, run `./build.sh`, which uses
the system libusb. ATK's `get_atkdaq.bat` clones this repo into
`vendor\atkdaq`, builds it, and reports the result. It uses a sibling checkout
at `D:\Analyst_Toolkit\atkdaq` if there is one.

Tests (Python 3.10+, numpy, pytest): `python -m pytest tests -q`. That runs
the frame-format tests, the C-vs-numpy cross-checks and the end-to-end tests
against the synthetic device. None of them needs the hardware.

## First run with the KrakenSDR

These are the things that will go wrong on a new machine, in the order they
will go wrong:

1. **Power.** Supply 5 V ≥ 2.4 A on the Kraken's power input from a proper
   supply. A laptop USB port browns it out under load. The symptom is lost
   samples on one channel, not an error.
2. **WinUSB on each of the five interfaces.** Run Zadig (ATK's folder has
   `zadig-2.9.exe`), then *Options → List All Devices*. The Kraken shows five
   RTL2838 entries (Bulk-In, Interface 0), one per channel. Select each in
   turn and *Replace Driver* with **WinUSB**. Doing one is the classic mistake:
   the others then fail to open. Re-run Zadig after moving the Kraken to a
   different USB port.
3. **A dedicated USB root port.** 24 MB/s crosses the Kraken's internal hub.
   Put nothing else on its root port: not the bladeRF, not the GPS puck, not a
   webcam.
4. `bin\atkdaq.exe enum` lists what libusb can see. You should get five
   devices, serials 1000–1004.
5. `bin\atkdaq.exe probe` gives the one-screen report: serials, a sync and
   calibration on the noise source, throughput, alignment, clipping, and a
   verdict line.

## Using it

```
atkdaq [run] [options]             stream frames to stdout (ATK's use)
atkdaq record FILE.atkq [options]  the same stream into a file (--seconds/--frames)
atkdaq play FILE.atkq [--loop] [--fast] [--speed X]
atkdaq probe [--seconds N] [--retune-test] [--t1 MINUTES]
atkdaq enum | help-config | version

options: --config FILE  --set KEY=VALUE  --device kraken|synth  --fc HZ
         --gain TENTHS  --mode static|mobile  --frames N  --seconds S
         --out FILE  --no-stdin
```

Settings come from a `key = value` file (`atkdaq.example.conf`) and then from
`--set`. The command line always wins. **An unknown key is refused with its
line number.** A misspelt setting that is silently ignored is one you believe
is in force when it is not. `atkdaq help-config` lists every key.

**Commands on stdin**, one per line: `tune <hz>`, `gain <tenths_db>`, `sync`,
`cal`, `check`, `noise on|off|auto`, `mode static|mobile`, `quiet`,
`interval sync_check|cal <s>`, `status`, `quit`. EOF on stdin also quits.

**Status on stderr**, one line per event, readable by both programs and
people:

```
ATKDAQ event=sync passport=1 delays=0,-17,5,-120,-3 changed=1 frac=0.0000,0.2101,... w_db=... w_deg=... spread_deg=0.160 ptn_db=30.1
ATKDAQ event=misaligned channels_samples=2:4946 since=7208955 from_s0=8640000 msg="..."
ATKDAQ event=status t=4.2 frames=81 dropped=0 overruns=0 seq=81 cal_state=calibrated ...
```

The events are `start device streaming noise proc_start sync cal check
proc_fail delays_moved misaligned common_loss overrun stall retune gain mode
interval status error fault exit`. `python/atkdaq/client.py` `parse_status`
parses them.

**Exit codes:**

| code | meaning |
|---|---|
| 0 | normal end: quit, EOF on stdin, a limit reached, or the consumer closed the pipe |
| 2 | usage or configuration error (the message names the setting) |
| 3 | device missing or could not be opened (the message names the serial) |
| 4 | the device opened but would not stream |
| 5 | device fault while streaming (unplugged, USB error) |
| 7 | internal error |

## The stream

`include/atkdaq_frame.h` is the contract; read its header comment. In short:

- A frame is a 96-byte fixed header, then one 32-byte record per channel,
  then `n_channels × samples_per_ch` int8 I/Q pairs, **channel-major**. Every
  header carries a CRC-32. A reader that loses its place skips forward to
  the next `ATKQ` whose CRC checks.
- **Delays are applied; weights are not.** Payload sample *j* of every
  channel was digitised within one sample period of the others. Each channel
  record reports two numbers the consumer must apply: the complex weight `w`
  **and** the fractional delay `frac_delay`. The fractional delay is a phase
  slope across the band. At 2.4 MSPS a 0.3-sample residual is 22° on a
  signal 500 kHz off centre, so applying the weight alone is right only at
  band centre. The correction for a signal at baseband offset f0 is
  `w · exp(+j2π·f0·frac/fs)`.
- Flags say what not to trust: `NOISE_ON`, `MISALIGNED`, `RETUNE`,
  `CAL_STALE`, `DISCONTINUITY`, `OVERRUN`, `CLIPPED`, and `SYNTHETIC` (set
  on every frame from the synthetic device, so a fixture is never mistaken
  for a capture). `passport` increments on every sync and every cal, and a
  consumer records it with every bearing.
- A stalled consumer costs frames, never alignment. `seq` skips, and
  `stream_sample0` still counts the samples that were dropped.

## Testing without the hardware: the synthetic device

`--device synth` is a synthetic coherent receiver behind the same device
interface as the Kraken. It lets you plant, with `--set synth.*=...`:

- per-channel stream start offsets (the integer delays sync must find);
- per-channel receiver gain, phase and fractional delay (what cal must find);
- a common noise source switched in place of the antennas;
- on-air emitters with per-element steering phases (after calibration, these
  must be the only phases left);
- USB arrival jitter;
- sample loss on one channel at a planted time;
- retunes that re-randomise every tuner phase, optionally shifting the
  stream offsets as well.

With `synth.fast=1` it runs as fast as the consumer reads. Every decision is
made by sample index rather than wall clock, so everything behaves exactly as
it does in real time. Examples:

```
atkdaq run --device synth --set synth.fast=1 --set synth.start=0,17,-5,120,3 ^
    --set synth.phase_deg=0,40,-100,170,20 --frames 100 > synth.atkq
atkdaq probe --device synth --retune-test
```

`tests/fixtures/synth_240k.atkq` is a small recording made this way (0.4 s at
240 kHz, with an emitter). atkdf's tests use it to hold atkdf's independent
reader to this program's writer.

## Untested until the Kraken arrives

The synthetic device proves the program's logic. These are the things only
the hardware can prove, and T1/T2 in the plan answer them:

- **The fork's GPIO numbering on the real board.** The noise source is taken
  to be GPIO 0 of channel 0's RTL2832U and channel m's bias tee GPIO m+1,
  following Heimdall's usage (`noise_gpio` is configurable). The first probe
  run will show it: *"noise seen: coherence ..."* near 1.0 means the switch
  works.
- **Real settle times.** Tuner lock after a retune and the noise switch
  transient are covered by `retune_settle_ms` / `settle_ms` (50 / 40 ms).
- **USB timing on Windows.** A channel losing samples is detected from the
  transfer arrival times. The floor is the arrival jitter, a few hundred
  microseconds, so hundreds of samples; below that, the periodic
  noise-source check catches it (`drops.c` explains). The probe reports
  what your machine does.
- **§6 of the plan, whether a retune keeps the integer delays.** The program
  learns this itself: three retunes that keep the delays and it switches to
  cal-only retunes. One that moves them and it never trusts that again.
  `atkdaq probe --retune-test` measures it deliberately.
- **MSVC itself.** The Windows build is verified with MinGW-w64 and the whole
  test suite passes against that `.exe` under wine
  (`tests/check_windows_build.sh`). `build.bat` on your machine is the final
  word.

## Layout

```
include/atkdaq_frame.h   THE WIRE FORMAT
include/atkdaq_core.h    the computing core as a library (for the cross-checks)
src/main.c               command line, subcommands, exit codes
src/daq.c                the engine: rings -> aligned frames -> writer; the procedures
src/sync.c cal.c         integer delays; weights + fractional delays + residual
src/drops.c clock.c      per-channel loss vs the common mode; the lower-envelope clock
src/sched.c              the calibration policy (static / mobile, the §6 learner)
src/ring.c frame.c       SPSC byte ring with absolute positions; header, CRC, conversion
src/control.c config.c   stdin commands; settings
src/plat.c log.c         the only OS-specific code; status lines
src/probe.c play.c       the one-screen report; replay
src/devices/kraken.c     the KrakenSDR through the fork
src/devices/synth.c      the synthetic coherent receiver
python/atkdaq/           frame parser, numpy twins, client, replay (GPL, see above)
tests/                   test_smoke.c, test_frames.py, test_cross_check.py,
                         test_end_to_end.py, check_windows_build.sh, fixtures/
vendor/                  librtlsdr (krakenrf, pinned), libusb 1.0.29, pocketfft,
                         win32/pthread.h (atkdaq's shim for librtlsdr on Windows)
bench/bench_throughput.py
```
"# atkdaq" 
