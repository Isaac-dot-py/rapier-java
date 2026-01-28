# PowerShell script to run Rapier Java examples
param(
    [Parameter(Position=0)]
    [string]$ExampleName
)

$ErrorActionPreference = "Stop"

# Check if example name is provided
if ([string]::IsNullOrWhiteSpace($ExampleName)) {
    Write-Host "Usage: .\run-example.ps1 <ExampleClassName>" -ForegroundColor Yellow
    Write-Host "Available examples:"
    Write-Host "  BouncingBallExample" -ForegroundColor Cyan
    Write-Host "  MultiObjectExample" -ForegroundColor Cyan
    Write-Host "  ComprehensiveExample" -ForegroundColor Cyan
    exit 1
}

$exampleClass = "com.rapier.example.$ExampleName"

# Get Maven classpath using a temporary file (more reliable than stdout capture)
$tempFile = New-TemporaryFile
try {
    mvn dependency:build-classpath -q -DincludeScope=runtime "-Dmdep.outputFile=$($tempFile.FullName)" | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to get Maven classpath"
    }
    $mavenClasspath = (Get-Content $tempFile.FullName -Raw).Trim()
} finally {
    Remove-Item $tempFile -ErrorAction SilentlyContinue
}

# Build full classpath - use semicolon for Windows
$classpath = "target/classes;example/target/classes;$mavenClasspath"

# Run the example
java -cp $classpath $exampleClass
