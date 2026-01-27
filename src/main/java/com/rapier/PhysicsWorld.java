package com.rapier;

import com.sun.jna.Native;
import com.sun.jna.ptr.DoubleByReference;
import java.io.File;

/**
 * The main physics world that manages all physics simulation.
 * This is the entry point for using Rapier in Java.
 */
public class PhysicsWorld {
    private final long worldHandle;
    private final RapierNative nativeLib;
    private boolean destroyed = false;
    
    /**
     * Creates a new physics world with the specified gravity.
     * @param gravityX The x component of gravity
     * @param gravityY The y component of gravity
     */
    public PhysicsWorld(double gravityX, double gravityY) {
        File libFile = NativeLibraryLoader.loadLibrary();
        nativeLib = Native.load(libFile.getAbsolutePath(), RapierNative.class);
        worldHandle = nativeLib.rapier_world_create(gravityX, gravityY);
    }
    
    /**
     * Creates a new physics world with gravity pointing downward.
     * @param gravity The magnitude of gravity (typically 9.81)
     */
    public PhysicsWorld(double gravity) {
        this(0.0, -gravity);
    }
    
    /**
     * Advances the simulation by one time step.
     */
    public void step() {
        if (destroyed) {
            throw new IllegalStateException("Cannot step a destroyed world");
        }
        nativeLib.rapier_world_step(worldHandle);
    }
    
    /**
     * Creates a dynamic rigid body at the specified position.
     * Dynamic bodies are affected by forces and gravity.
     * @param x The x coordinate
     * @param y The y coordinate
     * @return The created rigid body
     */
    public RigidBody createDynamicRigidBody(double x, double y) {
        if (destroyed) {
            throw new IllegalStateException("Cannot create rigid body in destroyed world");
        }
        long handle = nativeLib.rapier_rigid_body_create_dynamic(worldHandle, x, y);
        return new RigidBody(worldHandle, handle, nativeLib);
    }
    
    /**
     * Creates a dynamic rigid body at the specified position.
     * @param position The position vector
     * @return The created rigid body
     */
    public RigidBody createDynamicRigidBody(Vector2 position) {
        return createDynamicRigidBody(position.x, position.y);
    }
    
    /**
     * Creates a fixed (static) rigid body at the specified position.
     * Fixed bodies don't move and are not affected by forces.
     * @param x The x coordinate
     * @param y The y coordinate
     * @return The created rigid body
     */
    public RigidBody createFixedRigidBody(double x, double y) {
        if (destroyed) {
            throw new IllegalStateException("Cannot create rigid body in destroyed world");
        }
        long handle = nativeLib.rapier_rigid_body_create_fixed(worldHandle, x, y);
        return new RigidBody(worldHandle, handle, nativeLib);
    }
    
    /**
     * Creates a fixed (static) rigid body at the specified position.
     * @param position The position vector
     * @return The created rigid body
     */
    public RigidBody createFixedRigidBody(Vector2 position) {
        return createFixedRigidBody(position.x, position.y);
    }
    
    /**
     * Creates a cuboid (rectangle) collider attached to a rigid body.
     * @param body The rigid body to attach to
     * @param halfWidth Half of the width of the rectangle
     * @param halfHeight Half of the height of the rectangle
     * @return The created collider
     */
    public Collider createCuboidCollider(RigidBody body, double halfWidth, double halfHeight) {
        if (destroyed) {
            throw new IllegalStateException("Cannot create collider in destroyed world");
        }
        long handle = nativeLib.rapier_collider_create_cuboid(worldHandle, body.getHandle(), halfWidth, halfHeight);
        return new Collider(worldHandle, handle, nativeLib);
    }
    
    /**
     * Creates a ball (circle) collider attached to a rigid body.
     * @param body The rigid body to attach to
     * @param radius The radius of the circle
     * @return The created collider
     */
    public Collider createBallCollider(RigidBody body, double radius) {
        if (destroyed) {
            throw new IllegalStateException("Cannot create collider in destroyed world");
        }
        long handle = nativeLib.rapier_collider_create_ball(worldHandle, body.getHandle(), radius);
        return new Collider(worldHandle, handle, nativeLib);
    }
    
    /**
     * Destroys the physics world and frees native resources.
     * After calling this, the world cannot be used anymore.
     */
    public void destroy() {
        if (!destroyed) {
            nativeLib.rapier_world_destroy(worldHandle);
            destroyed = true;
        }
    }
    
    /**
     * Gets the internal handle for this world.
     * @return The world handle
     */
    public long getHandle() {
        return worldHandle;
    }
    
    @Override
    protected void finalize() throws Throwable {
        try {
            destroy();
        } finally {
            super.finalize();
        }
    }
}
