# rapier-java

Java 17 bindings for the [Rapier](https://github.com/dimforge/rapier) 2D physics engine with double precision.

## Features

- ✅ Full Java 17 bindings for Rapier 2D physics engine
- ✅ Double precision floating point (f64)
- ✅ **Data-based API** - direct JNA passthrough, no wrapper classes
- ✅ Handle-based access matching native Rapier patterns
- ✅ Native library automatically extracted and loaded from resources
- ✅ Cross-platform support (Linux, macOS, Windows)

## Requirements

- Java 17 or higher
- Maven 3.6 or higher
- Rust toolchain (for building native library)

## Building

### Option 1: Use Build Scripts (Recommended)

**Linux/macOS:**
```bash
./build.sh
```

**Windows PowerShell:**
```powershell
.\build.ps1
```

### Option 2: Manual Build Steps

#### Build the Native Library

```bash
cd native-lib
cargo build --release
```

#### Build the Java Library

```bash
mvn clean compile
```

## Usage

### Basic Example - Bouncing Ball

```java
import com.rapier.*;
import com.sun.jna.ptr.DoubleByReference;

// Get the native library interface
RapierNative rapier = Rapier.create();

// Create a physics world with gravity
long world = rapier.rapier_world_create(0.0, -9.81);

// Create a ground (fixed body)
long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
long groundCollider = rapier.rapier_collider_create_cuboid(world, ground, 50.0, 1.0);
rapier.rapier_collider_set_friction(world, groundCollider, 0.5);

// Create a ball (dynamic body)
long ball = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 10.0);
long ballCollider = rapier.rapier_collider_create_ball(world, ball, 0.5);
rapier.rapier_collider_set_restitution(world, ballCollider, 0.8);  // 80% bounce

// Simulate
DoubleByReference x = new DoubleByReference();
DoubleByReference y = new DoubleByReference();

for (int i = 0; i < 300; i++) {
    rapier.rapier_world_step(world);
    rapier.rapier_rigid_body_get_position(world, ball, x, y);
    System.out.printf("Ball at: (%.3f, %.3f)%n", x.getValue(), y.getValue());
}

// Clean up
rapier.rapier_world_destroy(world);
```

### Running the Examples

Compile the example classes:
```bash
javac -cp "target/classes:$(mvn dependency:build-classpath -q -DincludeScope=runtime -Dmdep.outputFile=/dev/stdout)" \
  -d example/target/classes example/src/main/java/com/rapier/example/*.java
```

Run the bouncing ball example:
```bash
java -cp "target/classes:example/target/classes:$(mvn dependency:build-classpath -q -DincludeScope=runtime -Dmdep.outputFile=/dev/stdout)" \
  com.rapier.example.BouncingBallExample
```

Or use the convenient scripts:

**Linux/macOS:**
```bash
./build.sh                             # Build everything
./run-example.sh BouncingBallExample   # Run bouncing ball example
./run-example.sh MultiObjectExample    # Run multi-object example
./run-example.sh ComprehensiveExample  # Run comprehensive test suite
./run-example.sh NewFeaturesExample    # Run new features demo
```

**Windows PowerShell:**
```powershell
.\build.ps1                            # Build everything
.\run-example.ps1 BouncingBallExample  # Run bouncing ball example
.\run-example.ps1 MultiObjectExample   # Run multi-object example
.\run-example.ps1 ComprehensiveExample # Run comprehensive test suite
```

## API Overview

This library provides a **data-based API** that directly maps to the native Rapier functions. All physics objects are represented as handles (`long` values) rather than wrapper classes.

### Entry Point

```java
RapierNative rapier = Rapier.create();
```

### World Functions

- `rapier_world_create(double gravity_x, double gravity_y)` - Create a world, returns handle
- `rapier_world_step(long world)` - Advance simulation by one timestep
- `rapier_world_destroy(long world)` - Clean up the world

### Rigid Body Functions

- `rapier_rigid_body_create_dynamic(long world, double x, double y)` - Create movable body
- `rapier_rigid_body_create_fixed(long world, double x, double y)` - Create static body
- `rapier_rigid_body_get_position(long world, long body, DoubleByReference x, DoubleByReference y)` - Query position
- `rapier_rigid_body_get_rotation(long world, long body)` - Query rotation
- `rapier_rigid_body_set_translation(long world, long body, double x, double y, boolean wake)` - Set position
- `rapier_rigid_body_set_linvel(long world, long body, double vx, double vy, boolean wake)` - Set velocity
- `rapier_rigid_body_apply_impulse(long world, long body, double ix, double iy, boolean wake)` - Apply impulse
- `rapier_rigid_body_set_linear_damping(long world, long body, double damping)` - Set damping
- `rapier_rigid_body_get_mass(long world, long body)` - Query mass

### Collider Functions

- `rapier_collider_create_cuboid(long world, long body, double half_w, double half_h)` - Create box
- `rapier_collider_create_ball(long world, long body, double radius)` - Create circle
- `rapier_collider_set_restitution(long world, long collider, double restitution)` - Set bounciness
- `rapier_collider_set_friction(long world, long collider, double friction)` - Set friction
- `rapier_collider_set_sensor(long world, long collider, boolean is_sensor)` - Make sensor
- `rapier_collider_set_density(long world, long collider, double density)` - Set density
- `rapier_collider_set_collision_groups(long world, long collider, int memberships, int filter)` - Set groups

## Architecture

The project consists of three main components:

1. **Rust FFI Layer** (`native-lib/`) - Exposes Rapier's functionality through C-compatible FFI
2. **Java Bindings** (`src/main/java/`) - JNA interface that directly maps to native functions
3. **Examples** (`example/`) - Demonstration programs showing usage

### Design Philosophy

- **No wrapper classes** - RigidBody, Collider, etc. are just handles (`long` values)
- **Direct passthrough** - All calls go straight to native code with no logic
- **User manages lifecycle** - You decide when to create/destroy objects
- **ECS-friendly** - Handle-based design works naturally with entity-component systems

## License

Apache 2.0 (same as Rapier)

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

