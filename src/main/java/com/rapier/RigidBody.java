package com.rapier;

import com.sun.jna.Native;
import com.sun.jna.ptr.DoubleByReference;

/**
 * Represents a rigid body in the physics simulation.
 * Rigid bodies are the entities that can move and be affected by forces.
 */
public class RigidBody {
    private final long worldHandle;
    private final long bodyHandle;
    private final RapierNative nativeLib;
    
    RigidBody(long worldHandle, long bodyHandle, RapierNative nativeLib) {
        this.worldHandle = worldHandle;
        this.bodyHandle = bodyHandle;
        this.nativeLib = nativeLib;
    }
    
    /**
     * Gets the current position of the rigid body.
     * @return The position as a Vector2
     */
    public Vector2 getPosition() {
        DoubleByReference x = new DoubleByReference();
        DoubleByReference y = new DoubleByReference();
        nativeLib.rapier_rigid_body_get_position(worldHandle, bodyHandle, x, y);
        return new Vector2(x.getValue(), y.getValue());
    }
    
    /**
     * Gets the current rotation angle of the rigid body in radians.
     * @return The rotation angle
     */
    public double getRotation() {
        return nativeLib.rapier_rigid_body_get_rotation(worldHandle, bodyHandle);
    }
    
    /**
     * Sets the translation (position) of the rigid body.
     * @param x The x coordinate
     * @param y The y coordinate
     * @param wakeUp Whether to wake up the body if it's sleeping
     */
    public void setTranslation(double x, double y, boolean wakeUp) {
        nativeLib.rapier_rigid_body_set_translation(worldHandle, bodyHandle, x, y, wakeUp);
    }
    
    /**
     * Sets the translation (position) of the rigid body.
     * @param position The position vector
     * @param wakeUp Whether to wake up the body if it's sleeping
     */
    public void setTranslation(Vector2 position, boolean wakeUp) {
        setTranslation(position.x, position.y, wakeUp);
    }
    
    /**
     * Sets the linear velocity of the rigid body.
     * @param vx The x component of velocity
     * @param vy The y component of velocity
     * @param wakeUp Whether to wake up the body if it's sleeping
     */
    public void setLinearVelocity(double vx, double vy, boolean wakeUp) {
        nativeLib.rapier_rigid_body_set_linvel(worldHandle, bodyHandle, vx, vy, wakeUp);
    }
    
    /**
     * Sets the linear velocity of the rigid body.
     * @param velocity The velocity vector
     * @param wakeUp Whether to wake up the body if it's sleeping
     */
    public void setLinearVelocity(Vector2 velocity, boolean wakeUp) {
        setLinearVelocity(velocity.x, velocity.y, wakeUp);
    }
    
    /**
     * Applies an impulse to the rigid body.
     * @param impulseX The x component of the impulse
     * @param impulseY The y component of the impulse
     * @param wakeUp Whether to wake up the body if it's sleeping
     */
    public void applyImpulse(double impulseX, double impulseY, boolean wakeUp) {
        nativeLib.rapier_rigid_body_apply_impulse(worldHandle, bodyHandle, impulseX, impulseY, wakeUp);
    }
    
    /**
     * Applies an impulse to the rigid body.
     * @param impulse The impulse vector
     * @param wakeUp Whether to wake up the body if it's sleeping
     */
    public void applyImpulse(Vector2 impulse, boolean wakeUp) {
        applyImpulse(impulse.x, impulse.y, wakeUp);
    }
    
    /**
     * Gets the internal handle for this rigid body.
     * @return The body handle
     */
    public long getHandle() {
        return bodyHandle;
    }
}
