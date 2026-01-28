package com.rapier;

import com.sun.jna.ptr.IntByReference;

/**
 * Represents a collider shape attached to a rigid body.
 * Colliders define the physical shape for collision detection.
 */
public class Collider {
    private final long worldHandle;
    private final long colliderHandle;
    private final RapierNative nativeLib;
    
    Collider(long worldHandle, long colliderHandle, RapierNative nativeLib) {
        this.worldHandle = worldHandle;
        this.colliderHandle = colliderHandle;
        this.nativeLib = nativeLib;
    }
    
    /**
     * Sets the restitution (bounciness) of the collider.
     * @param restitution The restitution coefficient (0.0 = no bounce, 1.0 = perfect bounce)
     */
    public void setRestitution(double restitution) {
        nativeLib.rapier_collider_set_restitution(worldHandle, colliderHandle, restitution);
    }
    
    /**
     * Sets the friction coefficient of the collider.
     * @param friction The friction coefficient (0.0 = no friction, higher = more friction)
     */
    public void setFriction(double friction) {
        nativeLib.rapier_collider_set_friction(worldHandle, colliderHandle, friction);
    }
    
    /**
     * Sets whether this collider is a sensor.
     * Sensor colliders detect intersections but don't generate contact forces.
     * @param isSensor true to make this a sensor, false for normal collision
     */
    public void setSensor(boolean isSensor) {
        nativeLib.rapier_collider_set_sensor(worldHandle, colliderHandle, isSensor);
    }
    
    /**
     * Checks if this collider is a sensor.
     * @return true if this collider is a sensor, false otherwise
     */
    public boolean isSensor() {
        return nativeLib.rapier_collider_is_sensor(worldHandle, colliderHandle);
    }
    
    /**
     * Sets the density of the collider.
     * The density affects the mass properties of the attached rigid body.
     * @param density The density value (mass per unit area)
     */
    public void setDensity(double density) {
        nativeLib.rapier_collider_set_density(worldHandle, colliderHandle, density);
    }
    
    /**
     * Gets the density of the collider.
     * @return The density value
     */
    public double getDensity() {
        return nativeLib.rapier_collider_get_density(worldHandle, colliderHandle);
    }
    
    /**
     * Sets the collision groups for this collider.
     * Collision groups control which colliders can interact with each other.
     * A collision can occur between two colliders only if:
     * - The memberships of collider A have at least one bit in common with the filter of collider B
     * - AND the memberships of collider B have at least one bit in common with the filter of collider A
     * 
     * Note: Java uses signed 32-bit integers, so use -1 (0xFFFFFFFF) for "all groups".
     * Bit patterns are preserved correctly when passed to the native layer.
     * 
     * @param memberships A bitmask indicating which groups this collider belongs to
     * @param filter A bitmask indicating which groups this collider can collide with
     */
    public void setCollisionGroups(int memberships, int filter) {
        nativeLib.rapier_collider_set_collision_groups(worldHandle, colliderHandle, memberships, filter);
    }
    
    /**
     * Gets the collision groups for this collider.
     * 
     * Note: Values may be negative in Java's signed representation when high bits are set.
     * Use Integer.toUnsignedLong() if you need the unsigned value for display purposes.
     * 
     * @return An array of two integers: [memberships, filter]
     */
    public int[] getCollisionGroups() {
        IntByReference outMemberships = new IntByReference();
        IntByReference outFilter = new IntByReference();
        nativeLib.rapier_collider_get_collision_groups(worldHandle, colliderHandle, outMemberships, outFilter);
        return new int[] { outMemberships.getValue(), outFilter.getValue() };
    }
    
    /**
     * Gets the internal handle for this collider.
     * @return The collider handle
     */
    public long getHandle() {
        return colliderHandle;
    }
}
