@echo off
setlocal
if "%~2"=="" (
  echo Usage: build_control_msgs.cmd ROS_DEPENDENCY_WORKSPACE CONTROL_MSGS_WORKSPACE
  exit /b 2
)
if not defined VSCMD_ARG_TGT_ARCH (
  echo Run from a developer command prompt with MSVC, Windows SDK and Ninja configured.
  exit /b 2
)
if not exist "%~1\ros2-windows\local_setup.bat" exit /b 2
if not exist "%~2" exit /b 2
call "%~1\ros2-windows\local_setup.bat"
if errorlevel 1 exit /b %errorlevel%
set "PYTHONIOENCODING=utf-8"
set "VSLANG=1033"
set "CMAKE_BUILD_PARALLEL_LEVEL=1"
set "MAKEFLAGS=-j1"
set "COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.summary"
cd /d "%~2"
colcon build --build-base build_ninja --merge-install --packages-select control_msgs --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
exit /b %errorlevel%
