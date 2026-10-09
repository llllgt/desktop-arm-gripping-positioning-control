param(
    [string]$RosHome=$env:DESKTOP_ARM_ROS_HOME,
    [string]$Pixi='',
    [string]$PythonPackages='',
    [string]$Output=''
)
$ErrorActionPreference='Stop'
if (-not $RosHome) { throw 'Pass -RosHome with the ROS dependency workspace, or set DESKTOP_ARM_ROS_HOME.' }
$RosHome=(Resolve-Path -LiteralPath $RosHome).Path
foreach ($relative in @('pixi.toml','ros2-windows/local_setup.bat','control_ws/install/local_setup.bat')) {
    if (-not (Test-Path -LiteralPath (Join-Path $RosHome $relative))) { throw "Missing ROS dependency: $relative" }
}
if (-not $Pixi) {
    $taskPixi=Get-Command pixi -ErrorAction SilentlyContinue
    if ($taskPixi) { $Pixi=$taskPixi.Source }
}
if (-not $Pixi -or -not (Test-Path -LiteralPath $Pixi)) { throw 'Pixi not found. Pass -Pixi with its executable path.' }
if (-not $PythonPackages) { $PythonPackages=Join-Path $PSScriptRoot '../.venv/Lib/site-packages' }
$PythonPackages=(Resolve-Path -LiteralPath $PythonPackages).Path
if (-not $Output) { $Output=Join-Path $PSScriptRoot '../results/ros2' }
$Output=[System.IO.Path]::GetFullPath($Output)
New-Item -ItemType Directory -Force -Path $Output | Out-Null
$settings=@{
    DESKTOP_ARM_ROS_HOME=$RosHome
    DESKTOP_ARM_PYTHON_PACKAGES=$PythonPackages
    DESKTOP_ARM_OUTPUT=$Output
    ROS_LOG_DIR=(Join-Path $Output 'ros-logs')
}
$previous=@{}
try {
    foreach ($key in $settings.Keys) {
        $previous[$key]=[Environment]::GetEnvironmentVariable($key,'Process')
        [Environment]::SetEnvironmentVariable($key,$settings[$key],'Process')
    }
    & $Pixi run --manifest-path (Join-Path $RosHome 'pixi.toml') cmd /d /c (Join-Path $PSScriptRoot 'ros_runtime.cmd')
    if ($LASTEXITCODE -ne 0) { throw "ROS verification failed ($LASTEXITCODE). See logs in the output directory." }
} finally {
    foreach ($key in $previous.Keys) { [Environment]::SetEnvironmentVariable($key,$previous[$key],'Process') }
}
