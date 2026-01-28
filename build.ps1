# PowerShell build script for Rapier Java Bindings
$ErrorActionPreference = "Stop"

Write-Host "Building Rapier Java Bindings..." -ForegroundColor Cyan
Write-Host ""

# Build native library
Write-Host "1. Building Rust native library..." -ForegroundColor Yellow
Push-Location native-lib
try {
    cargo build --release
    if ($LASTEXITCODE -ne 0) {
        throw "Cargo build failed"
    }
} finally {
    Pop-Location
}

# Copy native library to resources
Write-Host "2. Copying native library to resources..." -ForegroundColor Yellow
$resourceDir = "src/main/resources/native"
if (-not (Test-Path $resourceDir)) {
    New-Item -ItemType Directory -Path $resourceDir -Force | Out-Null
}

$copied = $false
$libPaths = @(
    "native-lib/target/release/rapier_java_ffi.dll",
    "native-lib/target/release/librapier_java_ffi.so",
    "native-lib/target/release/librapier_java_ffi.dylib"
)

foreach ($libPath in $libPaths) {
    if (Test-Path $libPath) {
        $destFile = Join-Path $resourceDir (Split-Path $libPath -Leaf)
        Copy-Item $libPath $destFile -Force
        Write-Host "  Copied: $libPath" -ForegroundColor Gray
        $copied = $true
    }
}

if (-not $copied) {
    Write-Warning "No native library found. Build may have failed."
}

# Build Java library
Write-Host "3. Building Java library..." -ForegroundColor Yellow
mvn clean compile
if ($LASTEXITCODE -ne 0) {
    throw "Maven build failed"
}

# Build examples
Write-Host "4. Building examples..." -ForegroundColor Yellow
$exampleTargetDir = "example/target/classes"
if (-not (Test-Path $exampleTargetDir)) {
    New-Item -ItemType Directory -Path $exampleTargetDir -Force | Out-Null
}

# Get Maven classpath
$classpath = mvn dependency:build-classpath -q -DincludeScope=runtime -Dmdep.outputFile="-" | Out-String
$classpath = $classpath.Trim()

# Build example classes - use semicolon for Windows classpath separator
$fullClasspath = "target/classes;$exampleTargetDir;$classpath"

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
