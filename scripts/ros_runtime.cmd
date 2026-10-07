@echo off
setlocal
call C:\pixi_ws\ros2-windows\local_setup.bat
call C:\pixi_ws\control_ws\install\local_setup.bat
set "DESKTOP_ARM_ROOT=%~dp0.."
set "PYTHONPATH=%~dp0..;%~dp0..\.venv\Lib\site-packages;%PYTHONPATH%"
set "PYTHONIOENCODING=utf-8"
python "%~dp0verify_ros.py"
exit /b %errorlevel%
