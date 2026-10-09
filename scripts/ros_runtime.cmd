@echo off
setlocal
if not defined DESKTOP_ARM_ROS_HOME (
  echo Set DESKTOP_ARM_ROS_HOME to the ROS dependency workspace.
  exit /b 2
)
call "%DESKTOP_ARM_ROS_HOME%\ros2-windows\local_setup.bat"
if errorlevel 1 exit /b %errorlevel%
call "%DESKTOP_ARM_ROS_HOME%\control_ws\install\local_setup.bat"
if errorlevel 1 exit /b %errorlevel%
if not defined DESKTOP_ARM_PYTHON_PACKAGES set "DESKTOP_ARM_PYTHON_PACKAGES=%~dp0..\.venv\Lib\site-packages"
if not defined DESKTOP_ARM_OUTPUT set "DESKTOP_ARM_OUTPUT=%~dp0..\results\ros2"
set "DESKTOP_ARM_ROOT=%~dp0.."
set "PYTHONPATH=%~dp0..;%DESKTOP_ARM_PYTHON_PACKAGES%;%PYTHONPATH%"
set "PYTHONIOENCODING=utf-8"
python "%~dp0verify_ros.py" --output "%DESKTOP_ARM_OUTPUT%"
exit /b %errorlevel%
