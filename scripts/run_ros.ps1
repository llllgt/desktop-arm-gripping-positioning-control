param([string]$RosHome='C:\pixi_ws', [string]$Pixi='')
$ErrorActionPreference='Stop'
if (-not $Pixi) {
    $taskPixi=Get-Command pixi -ErrorAction SilentlyContinue
    if ($taskPixi) { $Pixi=$taskPixi.Source }
    else { $Pixi=Join-Path $PSScriptRoot '../../tmp/runtime_downloads/pixi/pixi.exe' }
}
if (-not (Test-Path -LiteralPath $Pixi)) { throw 'Pixi not found. Pass -Pixi with its executable path.' }
if ($RosHome -ne 'C:\pixi_ws') { throw 'The configured local runtime is C:\pixi_ws. Edit ros_runtime.cmd for another installation.' }
& $Pixi run --manifest-path (Join-Path $RosHome 'pixi.toml') cmd /c (Join-Path $PSScriptRoot 'ros_runtime.cmd')
if ($LASTEXITCODE -ne 0) { throw "ROS verification failed ($LASTEXITCODE). See results/ros2 logs." }
