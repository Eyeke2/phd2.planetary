@echo off
setlocal EnableExtensions

rem Visual Studio 2026 requires CMake 4.2 or newer and the C++ desktop workload.
rem Honor an explicitly selected generator; otherwise use Visual Studio 2026.
if not defined CMAKE_GENERATOR set "CMAKE_GENERATOR=Visual Studio 18 2026"

rem Resolve paths relative to this script, even when invoked from elsewhere.
rem Pass --fresh once after changing Visual Studio versions to reset old tool paths.
cmake -Wno-dev -S "%~dp0." -B "%~dp0tmp" -G "%CMAKE_GENERATOR%" -A Win32 %*
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" (
    echo [ERROR] CMake configuration failed. Check the error above.
    echo After a Visual Studio upgrade, back up tmp\CMakeCache.txt and run run_cmake.bat --fresh.
)
exit /b %RESULT%
