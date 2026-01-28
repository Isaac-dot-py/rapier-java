# PowerShell build script for Rapier Java Bindings
$ErrorActionPreference = "Stop"

Write-Host "Building Rapier Java Bindings (Gradle)..." -ForegroundColor Cyan
Write-Host ""

$gradleBin = if (Test-Path ./gradlew) { "./gradlew" } else { "gradle" }

# Build Java library (triggers native build + resource copy via Gradle lifecycle)
Write-Host "1. Building Java + native artifacts via Gradle..." -ForegroundColor Yellow
& $gradleBin -DskipTests clean build
if ($LASTEXITCODE -ne 0) {
    throw "Gradle build failed"
}

# Build examples
Write-Host "2. Building examples..." -ForegroundColor Yellow
$exampleTargetDir = "example/target/classes"
if (-not (Test-Path $exampleTargetDir)) {
    New-Item -ItemType Directory -Path $exampleTargetDir -Force | Out-Null
}

$classpath = (& $gradleBin -q printRuntimeClasspath)
$fullClasspath = "build\classes\java\main;build\resources\main;" + $classpath + ";$exampleTargetDir"

javac -cp $fullClasspath -d $exampleTargetDir example/src/main/java/com/rapier/example/*.java
if ($LASTEXITCODE -ne 0) {
    throw "Example compilation failed"
}

Write-Host ""
Write-Host "Build complete!" -ForegroundColor Green
Write-Host ""
Write-Host "Run examples with:"
Write-Host "  .\run-example.ps1 BouncingBallExample" -ForegroundColor Cyan
Write-Host "  .\run-example.ps1 MultiObjectExample" -ForegroundColor Cyan
Write-Host "  .\run-example.ps1 ComprehensiveExample" -ForegroundColor Cyan
