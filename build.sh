#!/bin/bash
set -e

echo "Building Rapier Java Bindings..."
echo ""

# Build native library
echo "1. Building Rust native library..."
cd native-lib
cargo build --release
cd ..

# Copy native library to resources
echo "2. Copying native library to resources..."
mkdir -p src/main/resources/native
cp native-lib/target/release/librapier_java_ffi.so src/main/resources/native/ 2>/dev/null || \
cp native-lib/target/release/librapier_java_ffi.dylib src/main/resources/native/ 2>/dev/null || \
cp native-lib/target/release/rapier_java_ffi.dll src/main/resources/native/ 2>/dev/null || true

# Build Java library
echo "3. Building Java library..."
mvn clean compile

# Build examples
echo "4. Building examples..."
mkdir -p example/target/classes
javac -cp "target/classes:$(mvn dependency:build-classpath -q -DincludeScope=runtime -Dmdep.outputFile=/dev/stdout)" \
  -d example/target/classes example/src/main/java/com/rapier/example/*.java

echo ""
echo "Build complete!"
echo ""
echo "Run examples with:"
echo "  ./run-example.sh BouncingBallExample"
echo "  ./run-example.sh MultiObjectExample"
