@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
call C:\pixi_ws\ros2-windows\local_setup.bat
set "TASK_SDK=C:\pixi_ws\windows_sdk\microsoft.windows.sdk.cpp\c"
set "TASK_SDK_LIB=C:\pixi_ws\windows_sdk\microsoft.windows.sdk.cpp.x64\c"
set "INCLUDE=%TASK_SDK%\Include\10.0.22621.0\ucrt;%TASK_SDK%\Include\10.0.22621.0\shared;%TASK_SDK%\Include\10.0.22621.0\um;%INCLUDE%"
set "LIB=%TASK_SDK_LIB%\um\x64;%TASK_SDK_LIB%\ucrt\x64;%LIB%"
set "PATH=%TASK_SDK%\bin\10.0.22621.0\x64;%PATH%"
set "PYTHONIOENCODING=utf-8"
set "VSLANG=1033"
set "CMAKE_BUILD_PARALLEL_LEVEL=1"
set "MAKEFLAGS=-j1"
set "COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.summary"
cd /d C:\pixi_ws\control_ws
colcon build --build-base build_ninja --merge-install --packages-select control_msgs --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
exit /b %errorlevel%
