[CmdletBinding()]
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Release',
    [string]$SdkRoot,
    [string]$ToolchainRoot
)

$ErrorActionPreference = 'Stop'
$sourceDir = Join-Path $PSScriptRoot '../firmware'
$buildDir = Join-Path $sourceDir "build/$Configuration"
$configureArgs = @('-S', $sourceDir, '--preset', "CW32L010-$Configuration")
if ($SdkRoot) {
    $configureArgs += "-DCW32_SPL_ROOT=$((Resolve-Path -LiteralPath $SdkRoot).Path)"
}
if ($ToolchainRoot) {
    $configureArgs += "-DARM_GNU_TOOLCHAIN_ROOT=$((Resolve-Path -LiteralPath $ToolchainRoot).Path)"
}

& cmake @configureArgs
if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }

& cmake --build $buildDir --parallel
if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed.' }
