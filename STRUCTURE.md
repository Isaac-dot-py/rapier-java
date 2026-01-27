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
│
├── native-lib/                         # Rust FFI layer
│   ├── Cargo.toml                      # Rust project configuration
│   └── src/
│       └── lib.rs                      # C-compatible FFI bindings
│
├── src/main/                           # Java bindings
│   ├── java/com/rapier/
│   │   ├── PhysicsWorld.java          # Main physics world manager
│   │   ├── RigidBody.java             # Rigid body wrapper
│   │   ├── Collider.java              # Collider wrapper
│   │   ├── Vector2.java               # 2D vector math
│   │   ├── RapierNative.java          # JNA interface definitions
│   │   └── NativeLibraryLoader.java   # Native library loader
│   └── resources/native/
│       └── librapier_java_ffi.so      # Compiled native library
│
└── example/src/main/java/com/rapier/example/
    ├── BouncingBallExample.java       # Basic bouncing ball demo
    ├── MultiObjectExample.java        # Multiple objects demo
    └── ComprehensiveExample.java      # Complete feature test suite
```

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

**Functions** (24 total):
- `rapier_world_create()` - Initialize physics world
- `rapier_world_destroy()` - Clean up world
- `rapier_world_step()` - Advance simulation
- `rapier_rigid_body_create_dynamic()` - Create movable body
- `rapier_rigid_body_create_fixed()` - Create static body
- `rapier_rigid_body_get_position()` - Query position
- `rapier_rigid_body_get_rotation()` - Query rotation
- `rapier_rigid_body_set_translation()` - Set position
- `rapier_rigid_body_set_linvel()` - Set velocity
- `rapier_rigid_body_apply_impulse()` - Apply force
- `rapier_collider_create_cuboid()` - Add box shape
- `rapier_collider_create_ball()` - Add circle shape
- `rapier_collider_set_restitution()` - Set bounciness
- `rapier_collider_set_friction()` - Set friction

### 2. Java Bindings

#### PhysicsWorld.java
- Main entry point for physics simulation
- Manages native world lifecycle
- Factory methods for bodies and colliders
- Automatic resource cleanup

#### RigidBody.java
- Represents physical bodies
- Position and rotation queries
- Velocity control
- Impulse application

#### Collider.java
- Collision shape attached to bodies
- Material properties (friction, restitution)

#### Vector2.java
- 2D vector mathematics
- Addition, subtraction, scaling
- Length, normalization, dot product

#### NativeLibraryLoader.java
- Automatic native library extraction
- Cross-platform support
- Resource-based loading

#### RapierNative.java
- JNA interface definitions
- Direct mapping to C FFI functions

### 3. Example Applications

#### BouncingBallExample
**Purpose**: Demonstrates basic physics simulation

**Features**:
- Gravity
- Ground collision
- Ball with restitution (bouncing)
- Console output showing ball height over time

**Output**: Shows ball falling, bouncing, and settling

#### MultiObjectExample
**Purpose**: Shows multiple interacting objects

**Features**:
- Multiple bodies with different properties
- Various restitution values
- Impulse application
- Position tracking for multiple objects

**Output**: Parallel tracking of three objects

#### ComprehensiveExample
**Purpose**: Validates all implemented features

**Tests**:
1. Basic gravity and collision
2. Restitution effects (bouncy vs. dead)
3. Friction effects on sliding
4. Impulse application
5. Different collider shapes

**Output**: Pass/fail for each feature test

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

## API Stability

Current version: 1.0.0

- Core API is complete and functional
- Breaking changes possible before 2.0
- Based on Rapier 0.18 (older, stable version)

## Testing

All three examples serve as integration tests:
- BouncingBallExample: Basic functionality
- MultiObjectExample: Multi-body interactions
- ComprehensiveExample: Feature validation

To run all tests:
```bash
./run-example.sh BouncingBallExample
./run-example.sh MultiObjectExample
./run-example.sh ComprehensiveExample
```

Expected: All examples run without errors and produce expected physics behavior.

## Future Enhancements

Potential additions (not in current scope):
- 3D physics support
- Joint constraints
- Sensors and callbacks
- Query system (raycasting)
- Event handling
- Serialization
- More collider shapes
- Performance profiling tools

## License

Apache 2.0 - Same as Rapier physics engine
