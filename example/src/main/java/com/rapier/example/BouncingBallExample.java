package com.rapier.example;

import com.rapier.*;
import com.sun.jna.ptr.DoubleByReference;

/**
 * Example demonstrating a simple bouncing ball simulation using Rapier.
 * Uses the data-based API with handles instead of wrapper objects.
 */
public class BouncingBallExample {
    public static void main(String[] args) {
        System.out.println("=== Rapier Java 2D Physics Example ===");
        System.out.println("Simulating a bouncing ball...\n");
        
        // Get the native library interface
        RapierNative rapier = Rapier.create();
        
        // Create a physics world with gravity
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create a ground (fixed body)
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        long groundCollider = rapier.rapier_collider_create_cuboid(world, ground, 50.0, 1.0);
        rapier.rapier_collider_set_friction(world, groundCollider, 0.5);
        rapier.rapier_collider_set_restitution(world, groundCollider, 0.0);
        
        // Create a ball (dynamic body) at height 10.0
        long ball = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 10.0);
        long ballCollider = rapier.rapier_collider_create_ball(world, ball, 0.5);
        rapier.rapier_collider_set_restitution(world, ballCollider, 0.8);  // 80% bounce
        rapier.rapier_collider_set_friction(world, ballCollider, 0.3);
        
        // Simulate for 5 seconds (at 60 FPS = 300 steps)
        double dt = 1.0 / 60.0;
        int numSteps = 300;
        
        // For querying position
        DoubleByReference x = new DoubleByReference();
        DoubleByReference y = new DoubleByReference();
        
        System.out.println("Time(s)   | Position(x, y)        | Height");
        System.out.println("----------|----------------------|--------");
        
        for (int i = 0; i <= numSteps; i++) {
            // Print state every 30 steps (0.5 seconds)
            if (i % 30 == 0) {
                rapier.rapier_rigid_body_get_position(world, ball, x, y);
                double time = i * dt;
                System.out.printf("%7.2f   | %8.3f, %8.3f  | %6.3fm%n", 
                    time, x.getValue(), y.getValue(), y.getValue());
            }
            
            // Step the simulation
            rapier.rapier_world_step(world);
        }
        
        // Final position
        rapier.rapier_rigid_body_get_position(world, ball, x, y);
        double rotation = rapier.rapier_rigid_body_get_rotation(world, ball);
        System.out.println("\nSimulation complete!");
        System.out.printf("Final ball position: (%.3f, %.3f)%n", x.getValue(), y.getValue());
        System.out.printf("Final rotation: %.3f radians%n", rotation);
        
        // Clean up
        rapier.rapier_world_destroy(world);
        
        System.out.println("\n=== Example completed successfully ===");
    }
}
