#!/bin/bash
set -e

echo "Building Rapier Java Bindings (Gradle)..."
echo ""

GRADLE_BIN="./gradlew"
if [ ! -x "$GRADLE_BIN" ]; then
  GRADLE_BIN="gradle"
fi

echo "1. Building Java + native artifacts via Gradle..."
$GRADLE_BIN -DskipTests clean build

echo "2. Building examples..."
mkdir -p example/target/classes
CLASSPATH="$($GRADLE_BIN -q printRuntimeClasspath):build/classes/java/main:build/resources/main"
javac -cp "$CLASSPATH" -d example/target/classes example/src/main/java/com/rapier/example/*.java

echo ""
echo "Build complete!"
echo ""
echo "Run examples with:"
echo "  ./run-example.sh BouncingBallExample"
echo "  ./run-example.sh MultiObjectExample"
echo "  ./run-example.sh ComprehensiveExample"
