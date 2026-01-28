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

# Get Maven classpath
$mavenClasspath = mvn dependency:build-classpath -q -DincludeScope=runtime -Dmdep.outputFile="-" | Out-String
$mavenClasspath = $mavenClasspath.Trim()

# Build full classpath - use semicolon for Windows
$classpath = "target/classes;example/target/classes;$mavenClasspath"

# Run the example
java -cp $classpath $exampleClass
