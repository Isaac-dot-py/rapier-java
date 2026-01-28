# Publish artifacts to the project-local build/m2 repository (no installation to ~/.m2)
$ErrorActionPreference = "Stop"

$publishDir = Join-Path $PSScriptRoot "build/m2"
if (-not (Test-Path $publishDir)) {
    New-Item -ItemType Directory -Path $publishDir -Force | Out-Null
}

$repoUri = "file:///" + ($publishDir -replace "\\", "/")

Write-Host "Publishing rapier-java to $repoUri" -ForegroundColor Cyan

$gradleBin = if (Test-Path ./gradlew) { "./gradlew" } else { "gradle" }

& $gradleBin -DskipTests publish
if ($LASTEXITCODE -ne 0) {
    throw "Publish failed"
}

Write-Host "Artifacts published to $repoUri" -ForegroundColor Green
