param(
    [Parameter(Mandatory = $true)]
    [string]$FontPath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $FontPath -PathType Leaf)) {
    throw "Font file not found: $FontPath"
}

$fontDirectory = Join-Path (Split-Path -Parent $PSScriptRoot) 'fonts'
New-Item -ItemType Directory -Path $fontDirectory -Force | Out-Null

function New-DashboardBinaryFont {
    param(
        [int]$Size,
        [string]$Symbols,
        [string]$OutputName,
        [int16]$Ascent,
        [int16]$Descent
    )

    $destination = Join-Path $fontDirectory $OutputName
    $converterArgs = @(
        '--yes', 'lv_font_conv',
        '--size', $Size,
        '--bpp', '4',
        '--format', 'bin',
        '--font', $FontPath,
        '--symbols', $Symbols,
        '-o', $destination
    )
    & npx.cmd @converterArgs
    if ($LASTEXITCODE -ne 0) {
        throw "lv_font_conv failed for $OutputName"
    }

    # Subsetting removes unused ascender/descender glyphs, which makes the
    # generated line box shorter. Restore the configured dashboard metrics
    # without carrying all ASCII glyph bitmaps.
    [byte[]]$fontData = [System.IO.File]::ReadAllBytes($destination)
    if ($fontData.Length -lt 48 -or $fontData[41] -ne 1) {
        throw "Unexpected LVGL binary font header in $OutputName"
    }
    [BitConverter]::GetBytes($Ascent).CopyTo($fontData, 16)
    [BitConverter]::GetBytes($Descent).CopyTo($fontData, 18)
    [System.IO.File]::WriteAllBytes($destination, $fontData)

    $lineHeight = $Ascent - $Descent
    $baseLine = -$Descent
    Write-Host "${OutputName}: size=$Size line_height=$lineHeight base_line=$baseLine"
}

# Dashboard line-height and baseline values.
New-DashboardBinaryFont -Size 100 -Symbols ' -0123456789' `
    -OutputName 'eurostile-100.bin' -Ascent 76 -Descent -17
New-DashboardBinaryFont -Size 40 -Symbols ' -0123456789' `
    -OutputName 'eurostile-40.bin' -Ascent 31 -Descent -7
New-DashboardBinaryFont -Size 30 -Symbols ' %-0123456789ahkmpstw' `
    -OutputName 'eurostile-30.bin' -Ascent 24 -Descent -5
New-DashboardBinaryFont -Size 20 -Symbols ' -.0123456789c' `
    -OutputName 'eurostile-20.bin' -Ascent 16 -Descent -3
