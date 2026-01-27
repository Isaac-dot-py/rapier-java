package com.rapier;

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
     * Gets the internal handle for this collider.
     * @return The collider handle
     */
    public long getHandle() {
        return colliderHandle;
    }
}
