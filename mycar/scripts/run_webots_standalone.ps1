param(
    [string]$WebotsExe = ""
)

$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$worldPath = Join-Path $projectRoot "simulation\webots\worlds\Piste_CoVAPSy_2025a_standalone.wbt"

if (-not (Test-Path -LiteralPath $worldPath)) {
    throw "Standalone world not found at: $worldPath"
}

$candidates = @()
if ($WebotsExe) {
    $candidates += $WebotsExe
}

if ($env:WEBOTS_HOME) {
    $candidates += (Join-Path $env:WEBOTS_HOME "webotsw.exe")
    $candidates += (Join-Path $env:WEBOTS_HOME "webots.exe")
}

$candidates += @(
    "C:\Program Files\Webots\webotsw.exe",
    "C:\Program Files\Webots\webots.exe"
)

$resolvedWebots = $null
foreach ($candidate in $candidates) {
    if ($candidate -and (Test-Path -LiteralPath $candidate)) {
        $resolvedWebots = $candidate
        break
    }
}

if (-not $resolvedWebots) {
    $fromPath = Get-Command webotsw.exe, webots.exe, webotsw, webots -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($fromPath) {
        $resolvedWebots = $fromPath.Source
    }
}

if (-not $resolvedWebots) {
    throw "Webots executable not found. Install Webots or pass -WebotsExe 'C:\path\to\webotsw.exe'."
}

Write-Host "Launching Webots standalone world:"
Write-Host "  $worldPath"
Write-Host "Using:"
Write-Host "  $resolvedWebots"

Start-Process -FilePath $resolvedWebots -ArgumentList @($worldPath)
