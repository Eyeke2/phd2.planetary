PHD2 is the enhanced, second generation version of the popular PHD
guiding software from Stark Labs. PHD2 is free and open source.

Development and support forum:

  https://groups.google.com/forum/#!forum/open-phd-guiding

Web site:

  http://openphdguiding.org

Source code repository:

  https://github.com/OpenPHDGuiding

--

Windows build (Visual Studio 2026):

  Install CMake 4.2 or newer and Visual Studio's Desktop development with C++
  workload, alongside the project's existing dependencies.
  Run run_cmake.bat, then cmake --build tmp --config Release.
  The script can be launched from any directory and returns CMake's exit code.
  Set CMAKE_GENERATOR explicitly to use another installed Visual Studio version.

  When upgrading Visual Studio, first back up tmp/CMakeCache.txt, then run:
    run_cmake.bat --fresh
  This regenerates tmp/phd2.sln and resets cached compiler paths and custom CMake
  settings. Reapply any custom -D options on that command. Source files, dependency
  downloads, release ZIPs, and temporary worktrees are retained.
