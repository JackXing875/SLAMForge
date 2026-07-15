param(
    [Parameter(Mandatory = $true)]
    [string]$BuildDirectory,

    [Parameter(Mandatory = $true)]
    [string]$VcpkgInstalledDirectory,

    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,

    [Parameter(Mandatory = $true)]
    [string]$Version,

    [string]$Triplet = "x64-windows-release"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$buildRoot = (Resolve-Path $BuildDirectory).Path
$outputRoot = [System.IO.Path]::GetFullPath($OutputDirectory)
$packageName = "SLAMForge-Desktop-$Version-Windows-x64"
$packageRoot = Join-Path $outputRoot $packageName
$runtimeDirectory = Join-Path $VcpkgInstalledDirectory "$Triplet\bin"

New-Item -ItemType Directory -Force -Path $packageRoot | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $packageRoot "config") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $packageRoot "licenses\third-party") |
    Out-Null

$desktopExecutable = Join-Path $buildRoot "apps\Release\SLAMForge Desktop.exe"
$cliExecutable = Join-Path $buildRoot "apps\Release\slamforge_cli.exe"
if (-not (Test-Path $desktopExecutable) -or -not (Test-Path $cliExecutable)) {
    throw "Release desktop or CLI executable was not found in $buildRoot\apps\Release"
}

Copy-Item $desktopExecutable $packageRoot
Copy-Item $cliExecutable $packageRoot
Copy-Item (Join-Path $repositoryRoot "config\*.yaml") (Join-Path $packageRoot "config")
Copy-Item (Join-Path $repositoryRoot "LICENSE") (Join-Path $packageRoot "LICENSE.txt")
Copy-Item (Join-Path $repositoryRoot "THIRD_PARTY_NOTICES.md") $packageRoot
Copy-Item (Join-Path $PSScriptRoot "README.txt") $packageRoot

if (-not (Test-Path $runtimeDirectory)) {
    throw "vcpkg runtime directory does not exist: $runtimeDirectory"
}
Get-ChildItem $runtimeDirectory -Filter "*.dll" | Copy-Item -Destination $packageRoot

$vcpkgShare = Join-Path $VcpkgInstalledDirectory "$Triplet\share"
if (Test-Path $vcpkgShare) {
    Get-ChildItem $vcpkgShare -Directory | ForEach-Object {
        $copyright = Join-Path $_.FullName "copyright"
        if (Test-Path $copyright) {
            Copy-Item $copyright (Join-Path $packageRoot "licenses\third-party\$($_.Name).txt")
        }
    }
}

$deployer = (Get-Command "windeployqt.exe" -ErrorAction Stop).Source
& $deployer --release --dir $packageRoot (Join-Path $packageRoot "SLAMForge Desktop.exe")
if ($LASTEXITCODE -ne 0) {
    throw "windeployqt failed with exit code $LASTEXITCODE"
}

$qtRoot = Split-Path (Split-Path $deployer -Parent) -Parent
$qtLicenses = Join-Path $qtRoot "LICENSES"
if (Test-Path $qtLicenses) {
    Copy-Item $qtLicenses (Join-Path $packageRoot "licenses\Qt") -Recurse
}

New-Item -ItemType Directory -Force -Path $outputRoot | Out-Null
$archive = Join-Path $outputRoot "$packageName.zip"
if (Test-Path $archive) {
    Remove-Item $archive -Force
}
Compress-Archive -Path (Join-Path $packageRoot "*") -DestinationPath $archive `
    -CompressionLevel Optimal
Write-Host "Created $archive"
