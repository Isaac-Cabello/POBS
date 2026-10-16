param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $RosArguments
)

$rosRoot = 'C:\pixi_ws\ros2-windows'
$dependencyBin = "$rosRoot\.pixi\envs\default\Library\bin"
$env:Path = "$dependencyBin;$env:Path"
$env:ROS_LOG_DIR = 'C:\Users\kotha\pobs\.ros\log'
. "$rosRoot\local_setup.ps1"
Set-Location 'C:\Users\kotha\pobs'

if ($RosArguments.Count -gt 0) {
    & "$rosRoot\Scripts\ros2.exe" @RosArguments
    exit $LASTEXITCODE
}

Write-Host 'ROS 2 Lyrical is ready in the POBS folder.' -ForegroundColor Green
Write-Host 'Try: ros2 doctor --report'
