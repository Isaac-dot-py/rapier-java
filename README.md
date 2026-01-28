# rapier-java

Java 17 bindings for the [Rapier](https://github.com/dimforge/rapier) 2D physics engine with double precision.

## Features

- ✅ **Complete Rapier 2D API** - full coverage of Rapier functionality
- ✅ Double precision floating point (f64)
- ✅ **Data-based API** - direct JNA passthrough, no wrapper classes
- ✅ Handle-based access matching native Rapier patterns
- ✅ Native library automatically extracted and loaded from resources
- ✅ Cross-platform support (Linux, macOS, Windows)

### Supported Features

- **Rigid Bodies**: Dynamic, Fixed, Kinematic (position/velocity based)
- **Colliders**: Ball, Cuboid, Capsule, Segment, Triangle, Heightfield
- **Joints**: Revolute, Prismatic, Fixed, Rope, Spring
- **Physics Properties**: Mass, Inertia, Damping, Gravity Scale, CCD
- **Material Properties**: Friction, Restitution, Density
- **Coefficient Combine Rules**: Average, Min, Multiply, Max
- **Interaction Groups**: Collision groups, Solver groups
- **Motors**: Position/velocity targeting with configurable stiffness/damping

## Requirements

- Java 17 or higher
- Gradle 8+ (or use the Gradle Wrapper if added)
- Rust toolchain (for building native library)

## Building

### Option 1: Use Build Scripts (Recommended)

**Linux/macOS:**
```bash
./build.sh
```

**Windows PowerShell:**
```powershell
./build.ps1
```

### Option 2: Manual Build Steps (Gradle)

Build everything (Java + native) via Gradle (this triggers `cargo build --release` and bundles the native library into the JAR under `native/`):

```bash
gradle -DskipTests clean build
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

## Publishing (project-local, no ~/.m2 pollution)

Artifacts can be deployed to `build/m2` inside this repository so other projects can consume them without touching your global Maven cache.

**Linux/macOS:**
```bash
./publish.sh
```

**Windows PowerShell:**
```powershell
./publish.ps1
```

This runs `gradle publish` with a file-based repository at `build/m2`.

## Consuming from another project (Gradle example)

Add the repo and dependency to your client project's `build.gradle` (Groovy DSL):

```groovy
repositories {
  maven { url uri("../rapier-java/build/m2") } // adjust path as needed
  mavenCentral()
}

dependencies {
  implementation "com.rapier:rapier-java:1.0.0"
}
```

For Gradle 8+ with centralized repository declarations, place the `maven` block in `settings.gradle` inside `dependencyResolutionManagement { repositories { ... } }`.

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
./run-example.sh FullApiExample        # Run complete API demo (joints, etc.)
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
- `rapier_world_set_gravity(long world, double x, double y)` - Set gravity
- `rapier_world_set_timestep(long world, double dt)` - Set timestep
- `rapier_world_get_num_rigid_bodies(long world)` - Get body count

### Rigid Body Functions

- `rapier_rigid_body_create_dynamic(long world, double x, double y)` - Create movable body
- `rapier_rigid_body_create_fixed(long world, double x, double y)` - Create static body
- `rapier_rigid_body_create_kinematic_velocity_based(...)` - Create kinematic body
- `rapier_rigid_body_create_kinematic_position_based(...)` - Create kinematic body
- `rapier_rigid_body_get_position(...)` / `set_translation(...)` - Position
- `rapier_rigid_body_get_linvel(...)` / `set_linvel(...)` - Velocity
- `rapier_rigid_body_apply_impulse(...)` / `add_force(...)` - Forces
- `rapier_rigid_body_set_gravity_scale(...)` - Gravity scale
- `rapier_rigid_body_enable_ccd(...)` - Continuous collision detection
- `rapier_rigid_body_lock_rotations(...)` / `lock_translations(...)` - Lock axes

### Collider Functions

- `rapier_collider_create_cuboid(...)` - Create box shape
- `rapier_collider_create_ball(...)` - Create circle shape
- `rapier_collider_create_capsule(...)` - Create capsule shape
- `rapier_collider_create_segment(...)` - Create line segment
- `rapier_collider_create_triangle(...)` - Create triangle
- `rapier_collider_set_friction(...)` / `set_restitution(...)` - Material
- `rapier_collider_set_friction_combine_rule(...)` - Combine rule (0=Average, 1=Min, 2=Multiply, 3=Max)
- `rapier_collider_set_sensor(...)` - Make sensor (trigger)
- `rapier_collider_set_density(...)` / `set_mass(...)` - Mass properties
- `rapier_collider_set_collision_groups(...)` / `set_solver_groups(...)` - Groups

### Joint Functions

- `rapier_joint_create_revolute(...)` - Hinge joint (rotation only)
- `rapier_joint_create_prismatic(...)` - Slider joint (translation only)
- `rapier_joint_create_fixed(...)` - Fixed joint (no relative motion)
- `rapier_joint_create_rope(...)` - Distance limit joint
- `rapier_joint_create_spring(...)` - Spring-damper joint
- `rapier_joint_set_motor_velocity(...)` - Motor control
- `rapier_joint_set_limits(...)` - Joint limits

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

