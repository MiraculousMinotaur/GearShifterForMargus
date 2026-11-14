# Link submodules from the local lib/ folder into the user's Arduino sketchbook libraries directory
$repoRoot = Split-Path -Parent $PSScriptRoot
$libDir = Join-Path $repoRoot "lib"
$arduinoLibDir = Join-Path ([Environment]::GetFolderPath("MyDocuments")) "Arduino\libraries"

if (-not (Test-Path $arduinoLibDir)) {
    New-Item -ItemType Directory -Force -Path $arduinoLibDir | Out-Null
}

$libraries = @(
    "ArduinoJoystickWithFFBLibrary",
    "Adafruit_ADS1X15",
    "Adafruit_BusIO",
    "Adafruit_MCP23017"
)

foreach ($lib in $libraries) {
    $target = Join-Path $libDir $lib
    $link = Join-Path $arduinoLibDir $lib

    if (Test-Path $link) {
        Write-Host "Target already exists at $link. Refreshing..."
        cmd /c rmdir "$link" 2>$null
    }

    if (Test-Path $target) {
        cmd /c mklink /J "$link" "$target"
        Write-Host "Linked: $lib -> $link"
    } else {
        Write-Warning "Source directory not found: $target (did you run git submodule update --init --recursive?)"
    }
}

