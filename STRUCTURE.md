# Rapier Java Bindings - Project Structure

This document provides a complete overview of the project structure and implementation.

## Directory Structure

```
rapier-java/
├── README.md                           # Project documentation
├── LICENSE                             # Apache 2.0 license
├── .gitignore                          # Git ignore patterns
├── pom.xml                             # Maven project configuration
├── build.sh                            # Build script for entire project
├── run-example.sh                      # Script to run examples

├── native-lib/                         # Rust FFI layer
│   ├── Cargo.toml                      # Rust project configuration
│   └── src/
│       └── lib.rs                      # C-compatible FFI bindings

├── src/main/                           # Java bindings
│   ├── java/com/rapier/
│   │   ├── Rapier.java               # Entry point to get native interface
│   │   ├── RapierNative.java         # JNA interface definitions
│   │   └── NativeLibraryLoader.java  # Native library loader
│   └── resources/native/
│       └── librapier_java_ffi.so      # Compiled native library

└── example/src/main/java/com/rapier/example/
    ├── BouncingBallExample.java       # Basic bouncing ball demo
    ├── MultiObjectExample.java        # Multiple objects demo
    ├── ComprehensiveExample.java      # Complete feature test suite
    └── NewFeaturesExample.java        # Sensors, mass, damping demo
```

## Design Philosophy

This library follows a **data-based API** design, similar to the native Rapier engine:

- **No wrapper classes** for physics objects (rigid bodies, colliders)
- **Handle-based access** using `long` values representing native pointers
- **Direct function calls** with no hidden logic or state management
- **User manages lifecycle** of all physics objects

This approach:
- Minimizes overhead and abstraction
- Matches native Rapier patterns exactly
- Gives users full control over object management
- Simplifies integration with entity-component systems (ECS)

## Component Overview

### 1. Rust FFI Layer (`native-lib/src/lib.rs`)

**Purpose**: Exposes Rapier 2D physics engine through C-compatible FFI.

**Key Features**:
- World management (create, destroy, step simulation)
- Rigid body creation (dynamic, fixed)
- Collider creation (cuboid, ball)
- Property setters (position, velocity, friction, restitution)
- Property getters (position, rotation)
- Impulse application
- Sensor support (colliders that detect but don't block)
- Density and mass properties
- Angular inertia queries
- Collision groups for filtering interactions
- Linear and angular damping

### 2. Java Bindings

#### Rapier.java
- Entry point to get the native interface
- Singleton pattern for library loading
- `Rapier.create()` returns the `RapierNative` interface

#### RapierNative.java
- JNA interface definitions
- Direct mapping to all C FFI functions
- All functions operate on handles (long values)

#### NativeLibraryLoader.java
- Automatic native library extraction
- Cross-platform support
- Resource-based loading

### 3. Example Applications

All examples demonstrate the data-based API pattern:

```java
// Get the native interface
RapierNative rapier = Rapier.create();

// Create world (returns handle)
long world = rapier.rapier_world_create(0.0, -9.81);

// Create rigid body (returns handle)
long body = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 5.0);

// Create collider (returns handle)
long collider = rapier.rapier_collider_create_ball(world, body, 0.5);

// Query position using output parameters
DoubleByReference x = new DoubleByReference();
DoubleByReference y = new DoubleByReference();
rapier.rapier_rigid_body_get_position(world, body, x, y);

// Clean up
rapier.rapier_world_destroy(world);
```

## Build Process

### 1. Native Library Build
```bash
cd native-lib
cargo build --release
```
Produces: `native-lib/target/release/librapier_java_ffi.so` (Linux)

### 2. Java Compilation
```bash
mvn clean compile
```
Compiles all Java sources and copies resources

### 3. Example Compilation
```bash
javac -cp "target/classes:..." example/src/main/java/com/rapier/example/*.java
```

### 4. Automated Build
```bash
./build.sh
```
Builds everything in correct order

## Runtime

Native library is automatically extracted from resources to a temporary file and loaded via JNA.

No manual installation or path configuration required.

## Dependencies

### Rust
- rapier2d-f64 = 0.18 (2D physics with f64)
- Standard library only

### Java
- Java 17+
- JNA 5.13.0 (for native interface)
- Maven 3.6+ (build tool)

## Performance Characteristics

- **Double precision** throughout (64-bit floating point)
- **Optimized native code** (Rust with LTO)
- **Minimal JNA overhead** (direct function calls)
- **No garbage collection** of native objects (manual lifecycle)
- **Zero abstraction cost** - direct passthrough to native

## API Stability

Current version: 1.0.0

- Data-based API is stable
- Breaking changes possible before 2.0
- Based on Rapier 0.18 (older, stable version)

## Testing

All four examples serve as integration tests:
- BouncingBallExample: Basic functionality
- MultiObjectExample: Multi-body interactions
- ComprehensiveExample: Feature validation
- NewFeaturesExample: Sensors, density, mass, inertia, collision groups, and damping

To run all tests:
```bash
./run-example.sh BouncingBallExample
./run-example.sh MultiObjectExample
./run-example.sh ComprehensiveExample
./run-example.sh NewFeaturesExample
```

Expected: All examples run without errors and produce expected physics behavior.

## Future Enhancements

Potential additions (not in current scope):
- 3D physics support
- Joint constraints
- Sensor callbacks (intersection events)
- Query system (raycasting)
- Event handling
- Serialization
- More collider shapes
- Performance profiling tools

## License

Apache 2.0 - Same as Rapier physics engine
