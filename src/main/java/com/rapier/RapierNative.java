package com.rapier;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.DoubleByReference;

/**
 * Low-level JNA interface to the Rapier native library.
 * This interface maps directly to the C FFI functions.
 */
interface RapierNative extends Library {
    // World management
    long rapier_world_create(double gravity_x, double gravity_y);
    void rapier_world_destroy(long world_handle);
    void rapier_world_step(long world_handle);
    
    // RigidBody functions
    long rapier_rigid_body_create_dynamic(long world_handle, double x, double y);
    long rapier_rigid_body_create_fixed(long world_handle, double x, double y);
    boolean rapier_rigid_body_get_position(long world_handle, long body_handle, DoubleByReference out_x, DoubleByReference out_y);
    double rapier_rigid_body_get_rotation(long world_handle, long body_handle);
    boolean rapier_rigid_body_set_translation(long world_handle, long body_handle, double x, double y, boolean wake_up);
    boolean rapier_rigid_body_set_linvel(long world_handle, long body_handle, double vx, double vy, boolean wake_up);
    boolean rapier_rigid_body_apply_impulse(long world_handle, long body_handle, double impulse_x, double impulse_y, boolean wake_up);
    
    // Collider functions
    long rapier_collider_create_cuboid(long world_handle, long body_handle, double half_width, double half_height);
    long rapier_collider_create_ball(long world_handle, long body_handle, double radius);
    boolean rapier_collider_set_restitution(long world_handle, long collider_handle, double restitution);
    boolean rapier_collider_set_friction(long world_handle, long collider_handle, double friction);
}
