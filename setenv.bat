@echo off

setlocal enabledelayedexpansion

pushd "%~dp0" 

if exist .\updates.bat (
  echo Checking for updates...
  call .\updates.bat returnvalue
  if !returnvalue! == 1 (
    del .\updates.bat
  )
)

endlocal

set MAXIM_PATH=%CD%

echo.
echo Setting MAXIM_PATH environment variable...
echo %MAXIM_PATH%
echo.

echo Prepending MAXIM_PATH to Firmware PATH... 
set FIRMWARE_PATH=%MAXIM_PATH%\Libraries
echo %FIRMWARE_PATH%
echo.

echo Prepending MAXIM_PATH to TOOLCHAIN PATH... 
set TOOLCHAIN_PATH=%MAXIM_PATH%\Tools
echo %TOOLCHAIN_PATH%
echo.

echo "Export MAXIM_SBT_DIR and MAXIM_SBT_DEVICE variables"
::Default device is ME13
set MAXIM_SBT_DEVICE="MAX32570"
set MAXIM_SBT_DIR=%TOOLCHAIN_PATH%\SBT
echo %MAXIM_SBT_DIR%
echo %MAXIM_SBT_DEVICE%
echo

echo Prepending TOOLCHAIN_PATH to PATH... 
set PATH=%MAXIM_SBT_DIR%\bin;%TOOLCHAIN_PATH%\GNUTools\bin;%TOOLCHAIN_PATH%\xPack\riscv-none-embed-gcc\bin;%TOOLCHAIN_PATH%\MinGW\msys\1.0\bin;%TOOLCHAIN_PATH%\OpenOCD;%PATH%
echo %PATH%
echo.

popd
