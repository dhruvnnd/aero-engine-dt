param (
    [string]$Version = "dev",
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
$StageName  = "aero_engine_dt-$Version-win64"
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

Controls:
  Throttle/altitude/airspeed via keyboard or a connected gamepad.
  M           toggle raw model / noisy sensor feed
  G           show/hide the gamepad panel
  F           toggle fullscreen

Custom engine config:
  aero_engine_dt.exe --engine-spec configs\default.cfg
  (edit or copy that file, or use twin_config.exe to scaffold/validate one)

Headless tools (optional, run from a terminal):
  twin_sim.exe --list
  twin_sim.exe --profile cruise-climb --db runs\cc.db

Sync tool (optional, uploads logged runs to a server):
  aero-sync.exe -db runs\twin_sim.db -dry-run
  aero-sync.exe -db runs\twin_sim.db -server https://your-server

Requirements: Windows 10 or later, 64-bit. Nothing else to install --
SDL3.dll must stay next to aero_engine_dt.exe. aero-sync.exe needs
nothing else at all (no DLL, no runtime).
"@ | Out-File -FilePath (Join-Path $StageDir "README.txt") -Encoding utf8

if (Test-Path $ZipPath) { Remove-Item -Force $ZipPath }
Compress-Archive -Path "$StageDir\*" -DestinationPath $ZipPath

Write-Host "Done: $ZipPath" -ForegroundColor Green
