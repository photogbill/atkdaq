cmake_minimum_required(VERSION 3.16)
project(atkdaq C)

# atkdaq — coherent multi-channel acquisition (KrakenSDR). One program,
# atkdaq.exe; a small core library for the cross-check tests; a C smoke
# test. GPL-2.0 (it links the krakenrf fork of librtlsdr). See README.md.
#
# Windows: libusb is built HERE from vendor/libusb as its own DLL
# (libusb-1.0.dll, LGPL-2.1) beside atkdaq.exe — the same source list and
# definitions ATK's get_hackrf.bat already builds on Bill's machine.
# Elsewhere: the system libusb (pkg-config libusb-1.0), or none, in which
# case only the synthetic device is built.

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(ATKDAQ_KRAKEN "Build the KrakenSDR device (librtlsdr fork + libusb)" ON)
option(ATKDAQ_TESTS  "Build the C smoke test" ON)

set(OUT ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(CORE_SOURCES
  src/frame.c src/ring.c src/clock.c src/sync.c src/cal.c src/drops.c
  vendor/pocketfft/pocketfft.c)
set(APP_SOURCES
  src/main.c src/daq.c src/sched.c src/control.c src/config.c src/log.c
  src/plat.c src/play.c src/probe.c src/devices/synth.c src/devices/kraken.c)

if(MSVC)
  # NOT /fp:fast: the calibration fits compare phases and NaN-sensitive
  # arithmetic, and fast-math licenses the compiler to change both.
  set(ATKDAQ_CFLAGS /O2 /W3 /fp:precise)
  set(ATKDAQ_DEFS _CRT_SECURE_NO_WARNINGS)
else()
  set(ATKDAQ_CFLAGS -O2 -Wall -Wextra)
  set(ATKDAQ_DEFS _POSIX_C_SOURCE=200809L)
endif()

set(HAVE_RTL OFF)
if(ATKDAQ_KRAKEN)
  if(WIN32)
    set(USB ${CMAKE_CURRENT_SOURCE_DIR}/vendor/libusb/libusb)
    add_library(usb-1.0 SHARED
      ${USB}/core.c ${USB}/descriptor.c ${USB}/os/events_windows.c ${USB}/hotplug.c
      ${USB}/io.c ${USB}/strerror.c ${USB}/sync.c ${USB}/os/threads_windows.c
      ${USB}/os/windows_common.c ${USB}/os/windows_usbdk.c ${USB}/os/windows_winusb.c)
    target_include_directories(usb-1.0 PRIVATE vendor/libusb/msvc ${USB})
    target_compile_definitions(usb-1.0 PRIVATE _WIN32_WINNT=_WIN32_WINNT_VISTA
                               _CRT_SECURE_NO_WARNINGS NDEBUG)
    if(MSVC)
      target_link_options(usb-1.0 PRIVATE /DEF:${USB}/libusb-1.0.def)
    endif()
    set_target_properties(usb-1.0 PROPERTIES OUTPUT_NAME libusb-1.0 PREFIX ""
      RUNTIME_OUTPUT_DIRECTORY ${OUT} LIBRARY_OUTPUT_DIRECTORY ${OUT}
      ARCHIVE_OUTPUT_DIRECTORY ${OUT})
    set(USB_LINK usb-1.0)
    set(USB_INC ${USB})
    set(HAVE_RTL ON)
  else()
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
      pkg_check_modules(LIBUSB QUIET libusb-1.0)
    endif()
    if(LIBUSB_FOUND)
      set(USB_LINK ${LIBUSB_LINK_LIBRARIES})
      set(USB_INC ${LIBUSB_INCLUDE_DIRS})
      set(HAVE_RTL ON)
    else()
      message(STATUS "atkdaq: libusb-1.0 not found - synthetic device only")
    endif()
  endif()
endif()

if(HAVE_RTL)
  set(RTL ${CMAKE_CURRENT_SOURCE_DIR}/vendor/librtlsdr/src)
  add_library(rtlsdr_static STATIC
    ${RTL}/librtlsdr.c ${RTL}/tuner_e4k.c ${RTL}/tuner_fc0012.c
    ${RTL}/tuner_fc0013.c ${RTL}/tuner_fc2580.c ${RTL}/tuner_r82xx.c)
  target_compile_definitions(rtlsdr_static PUBLIC rtlsdr_STATIC)
  target_include_directories(rtlsdr_static PUBLIC vendor/librtlsdr/include
                             PRIVATE ${RTL} ${USB_INC})
  if(WIN32)
    # vendor/win32/pthread.h must be found BEFORE any other pthread.h
    target_include_directories(rtlsdr_static BEFORE PRIVATE vendor/win32)
    target_compile_definitions(rtlsdr_static PRIVATE WIN32 _CRT_SECURE_NO_WARNINGS)
    target_link_libraries(rtlsdr_static PUBLIC ${USB_LINK} ws2_32)
  else()
    target_link_libraries(rtlsdr_static PUBLIC ${USB_LINK})
  endif()
  if(MSVC)
    target_compile_options(rtlsdr_static PRIVATE /W1)
  else()
    target_compile_options(rtlsdr_static PRIVATE -w)
  endif()
endif()

add_executable(atkdaq ${CORE_SOURCES} ${APP_SOURCES})
target_include_directories(atkdaq PRIVATE include src vendor/pocketfft)
target_compile_definitions(atkdaq PRIVATE ${ATKDAQ_DEFS})
target_compile_options(atkdaq PRIVATE ${ATKDAQ_CFLAGS})
set_target_properties(atkdaq PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT})
if(HAVE_RTL)
  target_compile_definitions(atkdaq PRIVATE ATKDAQ_HAVE_RTLSDR)
  target_link_libraries(atkdaq PRIVATE rtlsdr_static)
endif()
if(NOT WIN32)
  target_link_libraries(atkdaq PRIVATE pthread m)
endif()

add_library(atkdaq_core SHARED ${CORE_SOURCES})
target_include_directories(atkdaq_core PRIVATE include src vendor/pocketfft)
target_compile_definitions(atkdaq_core PRIVATE ATKDAQ_CORE_SHARED ATKDAQ_CORE_BUILD ${ATKDAQ_DEFS})
target_compile_options(atkdaq_core PRIVATE ${ATKDAQ_CFLAGS})
set_target_properties(atkdaq_core PROPERTIES PREFIX "" OUTPUT_NAME atkdaq_core
  C_VISIBILITY_PRESET hidden
  RUNTIME_OUTPUT_DIRECTORY ${OUT} LIBRARY_OUTPUT_DIRECTORY ${OUT} ARCHIVE_OUTPUT_DIRECTORY ${OUT})
if(NOT WIN32)
  target_link_libraries(atkdaq_core PRIVATE m)
endif()

if(ATKDAQ_TESTS)
  add_executable(atkdaq_smoke tests/test_smoke.c src/sched.c src/config.c src/control.c
                 ${CORE_SOURCES})
  target_include_directories(atkdaq_smoke PRIVATE include src vendor/pocketfft)
  target_compile_definitions(atkdaq_smoke PRIVATE ${ATKDAQ_DEFS})
  set_target_properties(atkdaq_smoke PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT})
  if(NOT WIN32)
    target_link_libraries(atkdaq_smoke PRIVATE m)
  endif()
  enable_testing()
  add_test(NAME smoke COMMAND atkdaq_smoke)
endif()
