#!/bin/sh
# Build atkdaq on Linux/macOS into bin/: the program, the core library the
# cross-check tests load, and the C smoke test; then run the smoke test.
#
# The KrakenSDR device needs libusb-1.0 (pkg-config libusb-1.0); without it
# the build still succeeds with the synthetic device only, and says so.
# Windows uses build.bat, which also builds libusb from vendor/libusb.
set -e
cd "$(dirname "$0")"
mkdir -p bin
CC=${CC:-cc}
CFLAGS="-std=c99 -D_POSIX_C_SOURCE=200809L -O2 -Wall -Wextra -Iinclude -Isrc -Ivendor/pocketfft"
CORE="src/frame.c src/ring.c src/clock.c src/sync.c src/cal.c src/drops.c vendor/pocketfft/pocketfft.c"
APP="src/main.c src/daq.c src/sched.c src/control.c src/config.c src/log.c src/plat.c src/play.c src/probe.c src/devices/synth.c src/devices/kraken.c"

RTL=""
RTLFLAGS=""
LIBS="-lpthread -lm"
if pkg-config --exists libusb-1.0 2>/dev/null; then
  RTL="vendor/librtlsdr/src/librtlsdr.c vendor/librtlsdr/src/tuner_e4k.c vendor/librtlsdr/src/tuner_fc0012.c vendor/librtlsdr/src/tuner_fc0013.c vendor/librtlsdr/src/tuner_fc2580.c vendor/librtlsdr/src/tuner_r82xx.c"
  RTLFLAGS="-DATKDAQ_HAVE_RTLSDR -Drtlsdr_STATIC -Ivendor/librtlsdr/include -Ivendor/librtlsdr/src $(pkg-config --cflags libusb-1.0)"
  LIBS="$(pkg-config --libs libusb-1.0) $LIBS"
  echo "atkdaq: building with the KrakenSDR device (librtlsdr fork + system libusb)"
else
  echo "atkdaq: libusb-1.0 not found - building the synthetic device only"
fi

# librtlsdr is third-party C written for gnu99; its warnings are not ours to fix
if [ -n "$RTL" ]; then
  mkdir -p build/rtl
  for f in $RTL; do
    $CC -std=gnu99 -O2 -w -Drtlsdr_STATIC -Ivendor/librtlsdr/include -Ivendor/librtlsdr/src \
        $(pkg-config --cflags libusb-1.0) -c "$f" -o "build/rtl/$(basename "$f" .c).o"
  done
  RTLOBJ="build/rtl/*.o"
else
  RTLOBJ=""
fi

$CC $CFLAGS $RTLFLAGS $CORE $APP $RTLOBJ -o bin/atkdaq $LIBS
$CC $CFLAGS -fPIC -shared -fvisibility=hidden -DATKDAQ_CORE_SHARED -DATKDAQ_CORE_BUILD $CORE \
    -o bin/atkdaq_core.so -lm
$CC $CFLAGS tests/test_smoke.c src/sched.c src/config.c src/control.c $CORE -o bin/atkdaq_smoke -lm
./bin/atkdaq_smoke
