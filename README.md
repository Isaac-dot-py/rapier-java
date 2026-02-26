# Rapier Java JNI (2D f64)

Handle-based JNI bindings for the Rapier 2D `f64` physics engine. All native memory lives in Rust; Java only holds opaque `long` handles.

## Design highlights

- **Handle-based API**: Java only uses opaque `long` handles (no OO wrappers).
- **Rust-owned memory**: world memory is allocated and freed in Rust.
- **Safe JNI**: every JNI entry validates world handles and converts indices/generations.
- **Entities**: rigid bodies, colliders (including sensors), impulse joints, multibody joints.
- **Hooks & events**: optional callbacks and buffered collision/contact force events.

## Project layout

- `native/rapier2d_jni`: Rust `cdylib` with JNI entry points.
- `src/main/java/com/rapier`: Java JNI bindings (`NativeBindings`).
- `src/main/java/com/rapier/examples`: runnable examples.

## Quick start

1. Build the native library and Java classes.
2. Run the sample `BouncingBall` example.

```pwsh
./gradlew.bat runExample
```

## Usage sketch

```java
long world = NativeBindings.worldCreate();
try {
    NativeBindings.worldSetGravity(world, 0.0, -9.81);

    long body = NativeBindings.worldCreateRigidBody(
            world,
            NativeBindings.RIGID_BODY_DYNAMIC,
            0.0,
            10.0,
            0.0,
            0.0,
            0.0,
            0.0
    );

    int groups = 0xFFFF_FFFF;
    NativeBindings.worldCreateColliderBall(
            world,
            body,
            0.5,
            1.0,
            0.5,
            0.0,
            false,
            groups,
            groups
    );

    NativeBindings.worldStep(world, 1.0 / 60.0, 1);
} finally {
    NativeBindings.worldDestroy(world);
}
```

## Notes

- The JNI layer uses packed index + generation handles to avoid stale access.
- Joint accessors expose local anchors, limits, and motor stiffness.
- Collision queries are available via `worldCastRay`, `worldCastRayAndGetNormal`, and `worldQueryAabb`.
