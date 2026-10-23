@echo off
setlocal
call "%DESKTOP_ARM_ROS_HOME%\ros2-windows\local_setup.bat"
if errorlevel 1 exit /b %errorlevel%
call "%DESKTOP_ARM_ROS_HOME%\control_ws\install\local_setup.bat"
if errorlevel 1 exit /b %errorlevel%
set "DESKTOP_ARM_ROOT=%~dp0.."
set "PYTHONPATH=%~dp0..;%DESKTOP_ARM_PYTHON_PACKAGES%;%PYTHONPATH%"
set "PYTHONIOENCODING=utf-8"
set "contact_args="
if defined DESKTOP_ARM_CONTACT_STOP set "contact_args=--stop-during-scan"
if defined DESKTOP_ARM_TASK_CONFIG (
  python "%~dp0verify_ros_contact.py" --output "%DESKTOP_ARM_OUTPUT%" --config "%DESKTOP_ARM_TASK_CONFIG%" %contact_args%
) else (
  python "%~dp0verify_ros_contact.py" --output "%DESKTOP_ARM_OUTPUT%" %contact_args%
)
exit /b %errorlevel%
