@echo off
setlocal

@REM 填写自己的NDK路径，例如：C:\Users\xxxx\AppData\Local\Android\Sdk\ndk\23.1.7779620
set ANDROID_NDK_HOME="C:\Users\xxxx\AppData\Local\Android\Sdk\ndk\23.1.7779620"
set API=24
set TARGET=aarch64-linux-android

if not exist "%ANDROID_NDK_HOME%" (
    echo Error: NDK not found at %ANDROID_NDK_HOME%
    pause
    exit /b 1
)

set CC=%ANDROID_NDK_HOME%\toolchains\llvm\prebuilt\windows-x86_64\bin\%TARGET%%API%-clang

if not exist "%CC%" (
    echo Error: clang not found at %CC%
    pause
    exit /b 1
)

echo Compiling bat_capacity_correct...
echo Source: %~dp0bat_capacity_correct.c
echo Output: %~dp0..\bat_capacity_correct
echo.

"%CC%" ^
    -O2 ^
    -fPIE ^
    -pie ^
    -o "%~dp0..\bat_capacity_correct" ^
    "%~dp0bat_capacity_correct.c"

if %errorlevel% neq 0 (
    echo Build failed
    pause
    exit /b 1
)

echo Build completed
echo.
echo Output: %~dp0..\bat_capacity_correct
pause
