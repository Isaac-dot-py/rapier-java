package com.rapier;

import com.sun.jna.Native;
import java.io.File;

/**
 * Entry point for the Rapier physics library.
 * Provides access to the native library interface.
 * 
 * Example usage:
 * <pre>
 * import com.rapier.*;
 * import com.sun.jna.ptr.DoubleByReference;
 * 
 * RapierNative rapier = Rapier.create();
 * 
 * // Create a world with gravity
 * long world = rapier.rapier_world_create(0.0, -9.81);
 * 
 * // Create a dynamic rigid body
 * long body = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 5.0);
 * 
 * // Create a ball collider attached to the body
 * long collider = rapier.rapier_collider_create_ball(world, body, 0.5);
 * 
 * // Step the simulation
 * rapier.rapier_world_step(world);
 * 
 * // Query position
 * DoubleByReference x = new DoubleByReference();
 * DoubleByReference y = new DoubleByReference();
 * rapier.rapier_rigid_body_get_position(world, body, x, y);
 * 
 * // Clean up
 * rapier.rapier_world_destroy(world);
 * </pre>
 */
public final class Rapier {
    private static RapierNative instance = null;
    
    private Rapier() {}
    
    /**
     * Creates and returns the native library interface.
     * The library is loaded once and cached for subsequent calls.
     * 
     * @return The native library interface for calling Rapier functions
     */
    public static synchronized RapierNative create() {
        if (instance == null) {
            File libFile = NativeLibraryLoader.loadLibrary();
            instance = Native.load(libFile.getAbsolutePath(), RapierNative.class);
        }
        return instance;
    }
}
