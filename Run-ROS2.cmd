@echo off
setlocal

set "PIXI=C:\Users\kotha\AppData\Local\pixi\bin\pixi.exe"
set "ROS_ROOT=C:\pixi_ws\ros2-windows"

if not exist "%PIXI%" (
  echo Pixi was not found at %PIXI% 1>&2
  exit /b 1
)

if not exist "%ROS_ROOT%\local_setup.bat" (
  echo ROS 2 was not found at %ROS_ROOT% 1>&2
  exit /b 1
)

"%PIXI%" run --manifest-path "%ROS_ROOT%\pixi.toml" powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0ROS2-Environment.ps1" %*
