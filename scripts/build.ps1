# PowerShell build script for GearShifterForMargus
$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

Write-Host "==> Checking git submodules..."
if (-not (Test-Path "lib\Adafruit_ADS1X15\Adafruit_ADS1X15.h") -or -not (Test-Path "lib\ArduinoJoystickWithFFBLibrary\src\Joystick.h")) {
    Write-Host "Submodules not initialized. Initializing..."
    git submodule update --init --recursive
}

$arduinoCliCmd = Get-Command arduino-cli -ErrorAction SilentlyContinue
$arduinoCliExe = $null

if ($arduinoCliCmd) {
    $arduinoCliExe = $arduinoCliCmd.Source
} else {
    $bundledCli = "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
    if (Test-Path $bundledCli) {
        $arduinoCliExe = $bundledCli
    }
}

$pioCmd = Get-Command pio -ErrorAction SilentlyContinue

if ($arduinoCliExe) {
    Write-Host "==> Building with arduino-cli ($arduinoCliExe)..."
    & $arduinoCliExe compile main_sketch_GearStickForMargus @args
} elseif ($pioCmd) {
    Write-Host "==> Building with PlatformIO..."
    & $pioCmd.Source run @args
} else {
    Write-Error "Neither arduino-cli nor pio was found in PATH."
    Write-Host "Install arduino-cli: winget install Arduino.ArduinoCLI"
    Write-Host "Or install PlatformIO: pip install platformio"
    exit 1
}
