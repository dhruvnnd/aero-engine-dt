param (
    [Parameter(Position = 0)]
    [string]$SourceFile = "main.c",

    [Parameter(Position = 1)]
    [string]$OutputExe = "main.exe",

    [Parameter(Mandatory = $false)]
    [switch]$Release
)

# Static dependency paths
$IncludeDir  = "x86_64-w64-mingw32/include"
$LibDir      = "x86_64-w64-mingw32/lib"
$Library     = "SDL3"

# Initialize compiler arguments
$GccArgs = @(
    "-o", $OutputExe,
    $SourceFile,
    "-I$IncludeDir",
    "-L$LibDir",
    "-l$Library"
)

# Add optimization flags if -Release switch is used
if ($Release) {
    Write-Host "Release build enabled (O2 optimization, hiding console window)..." -ForegroundColor Yellow
    $GccArgs += @("-O2", "-mwindows")
}

Write-Host "Compiling $SourceFile -> $OutputExe..." -ForegroundColor Cyan

# Execute GCC
& gcc @GccArgs

# Check if compilation succeeded
if ($LASTEXITCODE -eq 0) {
    Write-Host "Build successful! Running $OutputExe..." -ForegroundColor Green
    
    # Ensure SDL3.dll can be found if it's in the bin directory
    $BinPath = Join-Path (Get-Location) "x86_64-w64-mingw32/bin"
    if (Test-Path $BinPath) {
        $env:Path = "$BinPath;$env:Path"
    }
} else {
    Write-Warning "Compilation failed. Check your errors above."
}
