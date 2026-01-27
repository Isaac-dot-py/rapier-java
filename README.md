# rapier-java

Java 17 bindings for the [Rapier](https://github.com/dimforge/rapier) 2D physics engine with double precision.

## Features

- ✅ Full Java 17 bindings for Rapier 2D physics engine
- ✅ Double precision floating point (f64)
- ✅ Clean, object-oriented API
- ✅ Native library automatically extracted and loaded from resources
- ✅ Cross-platform support (Linux, macOS, Windows)

## Requirements

- Java 17 or higher
- Maven 3.6 or higher
- Rust toolchain (for building native library)

## Building

### Build the Native Library

```bash
cd native-lib
cargo build --release
```

### Build the Java Library

```bash
mvn clean compile
```

## Usage

### Basic Example - Bouncing Ball

```java
import com.rapier.*;

// Create a physics world with gravity
PhysicsWorld world = new PhysicsWorld(0.0, -9.81);

// Create a ground (fixed body)
RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
Collider groundCollider = world.createCuboidCollider(ground, 50.0, 1.0);
groundCollider.setFriction(0.5);

// Create a ball (dynamic body)
RigidBody ball = world.createDynamicRigidBody(0.0, 10.0);
Collider ballCollider = world.createBallCollider(ball, 0.5);
ballCollider.setRestitution(0.8);  // 80% bounce

// Simulate
for (int i = 0; i < 300; i++) {
    world.step();
    Vector2 pos = ball.getPosition();
    System.out.printf("Ball at: (%.3f, %.3f)%n", pos.x, pos.y);
}

// Clean up
world.destroy();
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

Run the multi-object example:
```bash
java -cp "target/classes:example/target/classes:$(mvn dependency:build-classpath -q -DincludeScope=runtime -Dmdep.outputFile=/dev/stdout)" \
  com.rapier.example.MultiObjectExample
```

## API Overview

### PhysicsWorld

The main entry point for the physics simulation:

- `PhysicsWorld(double gravityX, double gravityY)` - Create a world with custom gravity
- `PhysicsWorld(double gravity)` - Create a world with downward gravity
- `step()` - Advance the simulation by one timestep
- `createDynamicRigidBody(double x, double y)` - Create a dynamic (movable) body
- `createFixedRigidBody(double x, double y)` - Create a fixed (static) body
- `createCuboidCollider(RigidBody body, double halfWidth, double halfHeight)` - Add a box collider
- `createBallCollider(RigidBody body, double radius)` - Add a circle collider
- `destroy()` - Clean up the world

### RigidBody

Represents a physical body in the simulation:

- `getPosition()` - Get current position as Vector2
- `getRotation()` - Get current rotation in radians
- `setTranslation(double x, double y, boolean wakeUp)` - Set position
- `setLinearVelocity(double vx, double vy, boolean wakeUp)` - Set velocity
- `applyImpulse(double x, double y, boolean wakeUp)` - Apply an impulse force

### Collider

Defines the shape for collision detection:

- `setRestitution(double restitution)` - Set bounciness (0.0 = no bounce, 1.0 = perfect bounce)
- `setFriction(double friction)` - Set friction (0.0 = no friction)

### Vector2

2D vector with double precision:

- `Vector2(double x, double y)` - Create a vector
- `add(Vector2 other)` - Vector addition
- `subtract(Vector2 other)` - Vector subtraction
- `scale(double scalar)` - Scalar multiplication
- `length()` - Get magnitude
- `normalize()` - Get unit vector
- `dot(Vector2 other)` - Dot product

## Architecture

The project consists of three main components:

1. **Rust FFI Layer** (`native-lib/`) - Exposes Rapier's functionality through C-compatible FFI
2. **Java Bindings** (`src/main/java/`) - JNA-based Java wrapper around the native library
3. **Examples** (`example/`) - Demonstration programs showing usage

## License

Apache 2.0 (same as Rapier)

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

