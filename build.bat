@echo off
setlocal enableextensions
rem  ==========================================================================
rem  Build atkdaq into bin\ and run the C smoke test.
rem
rem    bin\atkdaq.exe         the program (GPL-2.0; links the krakenrf librtlsdr)
rem    bin\libusb-1.0.dll     libusb 1.0.29, built here, kept as its own DLL
rem                           (LGPL-2.1) - atkdaq.exe needs it beside it
rem    bin\atkdaq_core.dll    the computing core, for the cross-check tests
rem    bin\atkdaq_smoke.exe   the C smoke test, run at the end
rem
rem  Exit 0 = everything built and the smoke test passed.
rem
rem  Needs the MSVC build tools - the same ones ATK's install.bat already
rem  requires. Run from a "x64 Native Tools Command Prompt", or let this
rem  script find vcvars64.bat itself. No CMake needed: cl.exe is driven
rem  directly, with the libusb source list and definitions ATK's
rem  get_hackrf.bat already builds successfully on this machine.
rem  build.bat /cmake uses CMake instead (%ATKDAQ_CMAKE%, PATH, or ATK's env).
rem  ==========================================================================
cd /d "%~dp0"
if not exist bin mkdir bin

where cl >nul 2>nul
if not errorlevel 1 goto :have_cl
rem  NOT inside a parenthesised block: the ")" in %ProgramFiles(x86)% closes
rem  the block early (ATK's install.bat learned this the hard way).
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto :have_cl
for /f "usebackq tokens=*" %%p in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set "VSPATH=%%p"
if defined VSPATH call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul
:have_cl
where cl >nul 2>nul
if errorlevel 1 goto :no_cl

if /i "%~1"=="/cmake" goto :with_cmake

set "L=vendor\libusb\libusb"
set "OBJ=build\obj"
if exist "%OBJ%" rd /s /q "%OBJ%"
mkdir "%OBJ%\usb"
mkdir "%OBJ%\rtl"
mkdir "%OBJ%\app"
mkdir "%OBJ%\core"
mkdir "%OBJ%\smoke"

echo atkdaq: [1/5] libusb-1.0.dll
cl /nologo /O2 /MD /W1 /LD /I"vendor\libusb\msvc" /I"%L%" /D_WIN32_WINNT=_WIN32_WINNT_VISTA /D_CRT_SECURE_NO_WARNINGS /DNDEBUG "%L%\core.c" "%L%\descriptor.c" "%L%\os\events_windows.c" "%L%\hotplug.c" "%L%\io.c" "%L%\strerror.c" "%L%\sync.c" "%L%\os\threads_windows.c" "%L%\os\windows_common.c" "%L%\os\windows_usbdk.c" "%L%\os\windows_winusb.c" /Fo"%OBJ%\usb\\" /Fe"bin\libusb-1.0.dll" /link /DEF:"%L%\libusb-1.0.def" /IMPLIB:"bin\libusb-1.0.lib"
if errorlevel 1 goto :fail

echo atkdaq: [2/5] librtlsdr, krakenrf fork (vendor\librtlsdr\PINNED.txt)
cl /nologo /O2 /MD /W1 /c /DWIN32 /D_CRT_SECURE_NO_WARNINGS /Drtlsdr_STATIC /I"vendor\win32" /I"vendor\librtlsdr\include" /I"vendor\librtlsdr\src" /I"%L%" vendor\librtlsdr\src\librtlsdr.c vendor\librtlsdr\src\tuner_e4k.c vendor\librtlsdr\src\tuner_fc0012.c vendor\librtlsdr\src\tuner_fc0013.c vendor\librtlsdr\src\tuner_fc2580.c vendor\librtlsdr\src\tuner_r82xx.c /Fo"%OBJ%\rtl\\"
if errorlevel 1 goto :fail

echo atkdaq: [3/5] atkdaq.exe
cl /nologo /O2 /MD /W3 /fp:precise /DATKDAQ_HAVE_RTLSDR /Drtlsdr_STATIC /D_CRT_SECURE_NO_WARNINGS /Iinclude /Isrc /Ivendor\pocketfft /Ivendor\librtlsdr\include src\*.c src\devices\*.c vendor\pocketfft\pocketfft.c "%OBJ%\rtl\*.obj" /Fo"%OBJ%\app\\" /Fe"bin\atkdaq.exe" /link bin\libusb-1.0.lib ws2_32.lib
if errorlevel 1 goto :fail

echo atkdaq: [4/5] atkdaq_core.dll
cl /nologo /O2 /MD /W3 /fp:precise /LD /DATKDAQ_CORE_SHARED /DATKDAQ_CORE_BUILD /D_CRT_SECURE_NO_WARNINGS /Iinclude /Isrc /Ivendor\pocketfft src\frame.c src\ring.c src\clock.c src\sync.c src\cal.c src\drops.c vendor\pocketfft\pocketfft.c /Fo"%OBJ%\core\\" /Fe"bin\atkdaq_core.dll"
if errorlevel 1 goto :fail

echo atkdaq: [5/5] smoke test
cl /nologo /O2 /MD /W3 /D_CRT_SECURE_NO_WARNINGS /Iinclude /Isrc /Ivendor\pocketfft tests\test_smoke.c src\sched.c src\config.c src\control.c src\frame.c src\ring.c src\clock.c src\sync.c src\cal.c src\drops.c vendor\pocketfft\pocketfft.c /Fo"%OBJ%\smoke\\" /Fe"bin\atkdaq_smoke.exe"
if errorlevel 1 goto :fail
goto :finish

:with_cmake
set "CMAKE="
if defined ATKDAQ_CMAKE if exist "%ATKDAQ_CMAKE%" set "CMAKE=%ATKDAQ_CMAKE%"
if not defined CMAKE (
  where cmake >nul 2>nul
  if not errorlevel 1 set "CMAKE=cmake"
)
if not defined CMAKE if exist "..\..\envs\atk_core\Scripts\cmake.exe" set "CMAKE=..\..\envs\atk_core\Scripts\cmake.exe"
if not defined CMAKE if exist "..\ATK\envs\atk_core\Scripts\cmake.exe" set "CMAKE=..\ATK\envs\atk_core\Scripts\cmake.exe"
if not defined CMAKE (
  echo atkdaq: /cmake asked for, and no cmake was found.
  exit /b 1
)
"%CMAKE%" -S . -B build\cmake -G Ninja -DCMAKE_BUILD_TYPE=Release >nul 2>nul
if errorlevel 1 (
  if exist build\cmake rd /s /q build\cmake
  "%CMAKE%" -S . -B build\cmake -DCMAKE_BUILD_TYPE=Release
  if errorlevel 1 goto :fail
)
"%CMAKE%" --build build\cmake --config Release
if errorlevel 1 goto :fail
rem  the Visual Studio generator puts outputs under Release\; copy them up
if exist bin\Release\atkdaq.exe copy /y bin\Release\*.* bin\ >nul

:finish
if exist vendor\libusb\COPYING copy /y vendor\libusb\COPYING bin\LICENSE-libusb.txt >nul
if exist vendor\librtlsdr\COPYING copy /y vendor\librtlsdr\COPYING bin\LICENSE-librtlsdr.txt >nul
if exist LICENSE copy /y LICENSE bin\LICENSE-atkdaq.txt >nul
del /q bin\*.exp 2>nul
if not exist bin\atkdaq.exe goto :fail
bin\atkdaq_smoke.exe
if errorlevel 1 goto :fail
bin\atkdaq.exe version
echo atkdaq: built bin\atkdaq.exe - next: bin\atkdaq.exe probe (with the Kraken connected)
exit /b 0

:no_cl
echo atkdaq: no C compiler found. Install the MSVC build tools ^(ATK's get_buildtools.bat^) and retry.
exit /b 1

:fail
echo atkdaq: BUILD FAILED - the compiler output above says why.
exit /b 1
