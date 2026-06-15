@echo off
setlocal enabledelayedexpansion

REM ============================================================
REM rawan-four-motion-plugin - Open Solution Script
REM This version does NOT require Visual Studio Insiders.
REM It only calls N8RO setup.cmd, then opens the solution using
REM your installed Visual Studio 2022/2026 with C++ tools.
REM ============================================================

set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR%"=="" (
    echo [Error] Failed to resolve script directory.
    pause
    exit /b 1
)

if "%SCRIPT_DIR:~-1%"=="\" (
    set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
)

set "N8RO_RELEASE_SETUP=%SCRIPT_DIR%\..\..\..\..\setup.cmd"
set "SOLUTION_FILE=%SCRIPT_DIR%\rawan-four-motion-plugin.slnx"

if not exist "%N8RO_RELEASE_SETUP%" (
    echo [Error] setup.cmd not found: %N8RO_RELEASE_SETUP%
    echo [Hint] This folder should be inside D:\N8RO_2\dev\samples\sim\rawan-four-motion-plugin
    pause
    exit /b 1
)

if not exist "%SOLUTION_FILE%" (
    echo [Error] Solution file not found: %SOLUTION_FILE%
    pause
    exit /b 1
)

call "%N8RO_RELEASE_SETUP%"
if errorlevel 1 (
    echo [Error] setup.cmd failed.
    pause
    exit /b 1
)

echo [OK] N8RO_RELEASE=%N8RO_RELEASE%
echo [OK] N8RO_RELEASE_USER_SIM_PLUGINS=%N8RO_RELEASE_USER_SIM_PLUGINS%
echo [OK] Opening solution: %SOLUTION_FILE%

REM First try vswhere to find any installed Visual Studio with C++ x64 tools.
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if exist "%VSWHERE%" (
    for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_INSTALL=%%i"
    if defined VS_INSTALL if exist "!VS_INSTALL!\Common7\IDE\devenv.exe" (
        echo [OK] Found Visual Studio: !VS_INSTALL!\Common7\IDE\devenv.exe
        start "" "!VS_INSTALL!\Common7\IDE\devenv.exe" "%SOLUTION_FILE%"
        exit /b 0
    )
)

REM Common fallback paths.
if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\IDE\devenv.exe" (
    start "" "%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\IDE\devenv.exe" "%SOLUTION_FILE%"
    exit /b 0
)

if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Professional\Common7\IDE\devenv.exe" (
    start "" "%ProgramFiles%\Microsoft Visual Studio\2022\Professional\Common7\IDE\devenv.exe" "%SOLUTION_FILE%"
    exit /b 0
)

if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Enterprise\Common7\IDE\devenv.exe" (
    start "" "%ProgramFiles%\Microsoft Visual Studio\2022\Enterprise\Common7\IDE\devenv.exe" "%SOLUTION_FILE%"
    exit /b 0
)

REM Final fallback: use Windows file association for .slnx.
echo [Warning] Could not find devenv.exe directly. Trying Windows file association.
start "" "%SOLUTION_FILE%"
exit /b 0
