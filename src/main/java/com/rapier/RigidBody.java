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
    
    /**
     * Sets additional mass for this rigid body.
     * This mass is added to the mass computed from attached colliders.
     * The total angular inertia will be scaled automatically based on this additional mass.
     * @param additionalMass The additional mass value
     * @param wakeUp Whether to wake up the body if it's sleeping
     */
    public void setAdditionalMass(double additionalMass, boolean wakeUp) {
        nativeLib.rapier_rigid_body_set_additional_mass(worldHandle, bodyHandle, additionalMass, wakeUp);
    }
    
    /**
     * Gets the total mass of this rigid body.
     * This includes mass from attached colliders and any additional mass.
     * @return The total mass
     */
    public double getMass() {
        return nativeLib.rapier_rigid_body_get_mass(worldHandle, bodyHandle);
    }
    
    /**
     * Gets the angular inertia (moment of inertia) of this rigid body.
     * Angular inertia determines resistance to angular acceleration.
     * @return The angular inertia value
     */
    public double getAngularInertia() {
        return nativeLib.rapier_rigid_body_get_angular_inertia(worldHandle, bodyHandle);
    }
    
    /**
     * Sets the linear damping coefficient of this rigid body.
     * Damping progressively slows down the body over time.
     * @param damping The linear damping coefficient (0.0 = no damping)
     */
    public void setLinearDamping(double damping) {
        nativeLib.rapier_rigid_body_set_linear_damping(worldHandle, bodyHandle, damping);
    }
    
    /**
     * Gets the linear damping coefficient of this rigid body.
     * @return The linear damping coefficient
     */
    public double getLinearDamping() {
        return nativeLib.rapier_rigid_body_get_linear_damping(worldHandle, bodyHandle);
    }
    
    /**
     * Sets the angular damping coefficient of this rigid body.
     * Angular damping progressively slows down rotational movement.
     * @param damping The angular damping coefficient (0.0 = no damping)
     */
    public void setAngularDamping(double damping) {
        nativeLib.rapier_rigid_body_set_angular_damping(worldHandle, bodyHandle, damping);
    }
    
    /**
     * Gets the angular damping coefficient of this rigid body.
     * @return The angular damping coefficient
     */
    public double getAngularDamping() {
        return nativeLib.rapier_rigid_body_get_angular_damping(worldHandle, bodyHandle);
    }
}
