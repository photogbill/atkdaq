#!/bin/sh
# Cross-compile the WINDOWS build with MinGW-w64 on Linux, into build/win/.
#
# Not the shipping build (that is MSVC, build.bat) — a check, runnable from
# the Linux sessions that write this code, that every Windows code path
# compiles and links: the Win32 half of plat.c, the pthread shim under
# librtlsdr, libusb's WinUSB backend, WriteFile/_setmode/QueryPerformance*.
# If wine is installed, the smoke test and a synthetic-device run execute
# too. MinGW is not MSVC, so a clean pass here is necessary, not sufficient;
# build.bat on the real machine is the final word.
set -e
cd "$(dirname "$0")/.."
CC=${MINGW_CC:-x86_64-w64-mingw32-gcc}
command -v "$CC" >/dev/null 2>&1 || { echo "no $CC - apt install mingw-w64"; exit 2; }
W=build/win
mkdir -p $W/usb $W/rtl $W/bin
cat > $W/usb/config.h <<'EOF'
/* config.h for the MinGW-w64 build CHECK only; MSVC uses msvc/config.h. */
#define DEFAULT_VISIBILITY __attribute__((visibility("default")))
#define ENABLE_LOGGING 1
#define PLATFORM_WINDOWS 1
#define PRINTF_FORMAT(a, b) __attribute__ ((__format__ (__printf__, a, b)))
EOF
L=vendor/libusb/libusb
for f in core descriptor hotplug io strerror sync os/events_windows os/threads_windows os/windows_common os/windows_usbdk os/windows_winusb; do
  $CC -O2 -w -I$W/usb -I$L -D_WIN32_WINNT=0x0600 -DNDEBUG -c $L/$f.c -o $W/usb/$(basename $f).o
done
$CC -shared -o $W/bin/libusb-1.0.dll $W/usb/*.o $L/libusb-1.0.def -Wl,--out-implib,$W/bin/libusb-1.0.dll.a
for f in librtlsdr tuner_e4k tuner_fc0012 tuner_fc0013 tuner_fc2580 tuner_r82xx; do
  $CC -std=gnu99 -O2 -w -DWIN32 -Drtlsdr_STATIC -Ivendor/win32 -Ivendor/librtlsdr/include \
      -Ivendor/librtlsdr/src -I$L -c vendor/librtlsdr/src/$f.c -o $W/rtl/$f.o
done
CORE="src/frame.c src/ring.c src/clock.c src/sync.c src/cal.c src/drops.c vendor/pocketfft/pocketfft.c"
APP="src/main.c src/daq.c src/sched.c src/control.c src/config.c src/log.c src/plat.c src/play.c src/probe.c src/devices/synth.c src/devices/kraken.c"
$CC -std=c99 -O2 -Wall -Wextra -Wvla -DATKDAQ_HAVE_RTLSDR -Drtlsdr_STATIC -Iinclude -Isrc \
    -Ivendor/pocketfft -Ivendor/librtlsdr/include $CORE $APP $W/rtl/*.o \
    -L$W/bin -lusb-1.0 -lws2_32 -o $W/bin/atkdaq.exe
$CC -std=c99 -O2 -Wall -Wextra -shared -DATKDAQ_CORE_SHARED -DATKDAQ_CORE_BUILD -Iinclude -Isrc \
    -Ivendor/pocketfft $CORE -o $W/bin/atkdaq_core.dll
$CC -std=c99 -O2 -Wall -Iinclude -Isrc -Ivendor/pocketfft tests/test_smoke.c src/sched.c \
    src/config.c src/control.c $CORE -o $W/bin/atkdaq_smoke.exe
echo "windows cross-build: OK ($W/bin)"
if command -v wine >/dev/null 2>&1; then
  export WINEDEBUG=-all
  wine $W/bin/atkdaq_smoke.exe
  wine $W/bin/atkdaq.exe version
fi
