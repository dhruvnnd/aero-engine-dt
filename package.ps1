param (
    [string]$Version = "",   # default: project VERSION from CMakeLists.txt
    [switch]$SkipBuild
)

# Builds a Release configuration and packages it into a single zip that
# other people can unzip and run -- no compiler, no SDL3 dev kit, no cmake
# required on their machine. The Release exe only pulls in Windows' own
# Universal CRT DLLs (api-ms-win-crt-*, part of Windows 10/11) plus
# SDL3.dll, which is bundled alongside it here.

$ErrorActionPreference = "Stop"
$RepoRoot   = $PSScriptRoot
$BuildDir   = Join-Path $RepoRoot "build-release"
if (-not $Version) {
    $m = Select-String -Path (Join-Path $RepoRoot "CMakeLists.txt") `
        -Pattern 'project\(\s*aero_engine_dt\s+VERSION\s+([0-9][0-9A-Za-z.\-]*)' |
        Select-Object -First 1
    if (-not $m) { throw "No -Version given and no project VERSION found in CMakeLists.txt" }
    $Version = $m.Matches[0].Groups[1].Value
}
$Version    = $Version.TrimStart('v')   # accept "v0.2.0" or "0.2.0"; the name adds the v
$StageName  = "aero_engine_dt-v$Version-win64"
$DistDir    = Join-Path $RepoRoot "dist"
$StageDir   = Join-Path $DistDir $StageName
$ZipPath    = Join-Path $DistDir "$StageName.zip"

$SyncExePath = Join-Path $BuildDir "aero-sync.exe"

if (-not $SkipBuild) {
    Write-Host "Configuring Release build..." -ForegroundColor Cyan
    cmake -S $RepoRoot -B $BuildDir -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
    if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

    Write-Host "Building aero_engine_dt, twin_sim, twin_config (Release)..." -ForegroundColor Cyan
    cmake --build $BuildDir --config Release --target aero_engine_dt twin_sim twin_config
    if ($LASTEXITCODE -ne 0) { throw "cmake build failed" }

    Write-Host "Building aero-sync (Go, static, no cgo)..." -ForegroundColor Cyan
    go build -C (Join-Path $RepoRoot "sync") -o $SyncExePath .
    if ($LASTEXITCODE -ne 0) { throw "go build failed" }
}

$ExePath = Join-Path $BuildDir "aero_engine_dt.exe"
$DllPath = Join-Path $BuildDir "SDL3.dll"
if (-not (Test-Path $ExePath)) { throw "Missing $ExePath -- build failed?" }
if (-not (Test-Path $DllPath)) { throw "Missing $DllPath -- SDL3 vendor kit not found next to the exe" }
if (-not (Test-Path $SyncExePath)) { throw "Missing $SyncExePath -- go build failed?" }

Write-Host "Staging distributable at $StageDir ..." -ForegroundColor Cyan
if (Test-Path $StageDir) { Remove-Item -Recurse -Force $StageDir }
New-Item -ItemType Directory -Force -Path $StageDir | Out-Null

Copy-Item $ExePath $StageDir
Copy-Item $DllPath $StageDir
Copy-Item (Join-Path $BuildDir "twin_sim.exe") $StageDir -ErrorAction SilentlyContinue
Copy-Item (Join-Path $BuildDir "twin_config.exe") $StageDir -ErrorAction SilentlyContinue
Copy-Item $SyncExePath $StageDir
Copy-Item (Join-Path $RepoRoot "configs") $StageDir -Recurse

@"
aero engine digital twin -- v$Version

Run it:
  Double-click aero_engine_dt.exe (or run it from a terminal).
  The Layout menu switches between window layouts (Overview, Trends,
  Systems, Input & Log, Configuration); View shows or hides any panel.
  Pick Layout > Overview if panels look scattered. Your arrangement is
  saved to aero_engine_dt_imgui.ini in the folder you launch from.

Controls (most are also on the Controls panel):
  UP/DN or W/S   throttle           PGUP/PGDN   altitude
  [ ]            airspeed           - =         OAT offset
  R              reset flight condition
  I / O          start / stop engine
  P / .          pause / single-step the simulation
  M              raw model / noisy sensor feed
  SPACE          acknowledge alarms (or click MASTER WARNING / CAUTION)
  G / L          gamepad panel / event log
  F              fullscreen
  A button on a connected gamepad also acknowledges alarms.

Simulation panels: Instruments, Cylinders, Environment, Trends, Cylinder
Trends, Event Log, Alarms, Sim (clock and speed), Controls, Fault Injection.

Recording:
  File > Start recording logs the session to runs\dashboard.db (created
  next to where you launch the app). aero-sync.exe can upload it.

Engine configs:
  File > Load engine spec... loads a spec file and restarts the simulation.
  The Engine Spec panel shows the running engine; the Spec Editor creates,
  edits, validates and saves specs. From a terminal:
  aero_engine_dt.exe --engine-spec configs\default.cfg
  twin_config.exe --new my_engine.cfg / --check my_engine.cfg

Headless tools (optional, run from a terminal):
  twin_sim.exe --list
  twin_sim.exe --profile cruise-climb --db runs\cc.db

Sync tool (optional, uploads logged runs to a server):
  aero-sync.exe -db runs\dashboard.db -dry-run
  aero-sync.exe -db runs\dashboard.db -server https://your-server

Known limitations:
  The load is a fixed placeholder (8 N*m), so RPM can pass the warning limit
  in normal running, and engines other than the default are not calibrated.

Requirements: Windows 10 or later, 64-bit. Nothing else to install --
SDL3.dll must stay next to aero_engine_dt.exe. aero-sync.exe needs
nothing else at all (no DLL, no runtime).
"@ | Out-File -FilePath (Join-Path $StageDir "README.txt") -Encoding utf8

if (Test-Path $ZipPath) { Remove-Item -Force $ZipPath }
Compress-Archive -Path "$StageDir\*" -DestinationPath $ZipPath

Write-Host "Done: $ZipPath" -ForegroundColor Green
