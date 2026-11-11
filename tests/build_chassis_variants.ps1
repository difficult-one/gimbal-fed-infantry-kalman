param(
    [string]$GccPath,
    [string]$MakeCommand = 'make',
    [string]$OutputRoot = 'build/variants'
)
$ErrorActionPreference = 'Stop'
$originalPath = $env:PATH
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    if ($GccPath) {
        $toolchain = (Resolve-Path -LiteralPath $GccPath).Path
        if (!(Test-Path -LiteralPath (Join-Path $toolchain 'arm-none-eabi-gcc.exe') -PathType Leaf)) {
            throw "ARM GCC toolchain not found in $toolchain"
        }
        # Prepend the bin directory so Make can invoke GCC even when the directory contains spaces.
        $env:PATH = "$toolchain$([IO.Path]::PathSeparator)$originalPath"
    } elseif (!(Get-Command arm-none-eabi-gcc -CommandType Application -ErrorAction SilentlyContinue)) {
        throw 'ARM GCC toolchain not found on PATH'
    }
    if ($OutputRoot -match '\s') {
        throw 'OutputRoot must be a path without spaces for this Makefile'
    }
    foreach ($board in 'ONE_BOARD', 'CHASSIS_BOARD', 'GIMBAL_BOARD') {
        foreach ($drive in 'OMNI_X', 'MECANUM', 'SWERVE') {
            # Separate object directories: changing compiler defines must never reuse other board objects.
            $directory = (Join-Path $OutputRoot "${board}_${drive}").Replace('\', '/')
            New-Item -ItemType Directory -Path $directory -Force | Out-Null
            $log = "$directory/build.log"
            & $MakeCommand -j4 'GCC_PATH=' "BUILD_DIR=$directory" "EXTRA_DEFS=-D$board -DCHASSIS_DRIVE_$drive" *> $log
            if ($LASTEXITCODE -ne 0) {
                Get-Content $log -Tail 40
                throw "Build failed: $board / $drive; see $log"
            }
            Write-Output "PASS ARM: $board / $drive"
        }
    }
} finally {
    $env:PATH = $originalPath
    Pop-Location
}
