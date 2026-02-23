@echo off
REM PlatformIO build and upload script for Teensy 4.1 on COM3

pio run -e teensy41_debug -t clean --upload-port COM3
if %ERRORLEVEL% neq 0 goto error

pio run -e teensy41_debug -t upload --upload-port COM3
if %ERRORLEVEL% neq 0 goto error

echo Done.
goto end

:error
echo Failed with error code %ERRORLEVEL%.
exit /b %ERRORLEVEL%

:end