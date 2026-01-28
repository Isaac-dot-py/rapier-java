package com.rapier.example;

import com.rapier.*;
import com.sun.jna.ptr.DoubleByReference;

/**
 * Example demonstrating multiple objects with different properties.
 * Uses the data-based API with handles instead of wrapper objects.
 */
public class MultiObjectExample {
    public static void main(String[] args) {
        System.out.println("=== Rapier Multi-Object Example ===");
        System.out.println("Simulating multiple falling objects...\n");
        
        // Get the native library interface
        RapierNative rapier = Rapier.create();
        
        // Create a physics world with gravity
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create ground
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        long groundCollider = rapier.rapier_collider_create_cuboid(world, ground, 100.0, 1.0);
        rapier.rapier_collider_set_restitution(world, groundCollider, 0.2);
        
        // Create a bouncy ball
        long bouncyBall = rapier.rapier_rigid_body_create_dynamic(world, -5.0, 15.0);
        long bouncyCollider = rapier.rapier_collider_create_ball(world, bouncyBall, 0.5);
        rapier.rapier_collider_set_restitution(world, bouncyCollider, 0.9);  // Very bouncy
        rapier.rapier_collider_set_friction(world, bouncyCollider, 0.1);
        
        // Create a heavy box
        long box = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 12.0);
        long boxCollider = rapier.rapier_collider_create_cuboid(world, box, 0.7, 0.7);
        rapier.rapier_collider_set_restitution(world, boxCollider, 0.3);  // Less bouncy
        rapier.rapier_collider_set_friction(world, boxCollider, 0.5);
        
        // Create another ball with impulse
        long pushedBall = rapier.rapier_rigid_body_create_dynamic(world, 5.0, 10.0);
        long pushedCollider = rapier.rapier_collider_create_ball(world, pushedBall, 0.4);
        rapier.rapier_collider_set_restitution(world, pushedCollider, 0.6);
        rapier.rapier_collider_set_friction(world, pushedCollider, 0.2);
        rapier.rapier_rigid_body_apply_impulse(world, pushedBall, -3.0, 2.0, true);  // Give it a push
        
        // For querying position
        DoubleByReference x = new DoubleByReference();
        DoubleByReference y = new DoubleByReference();
        
        // Simulate
        System.out.println("Time(s) | Bouncy Ball   | Box           | Pushed Ball");
        System.out.println("--------|---------------|---------------|---------------");
        
        double dt = 1.0 / 60.0;
        int numSteps = 240;  // 4 seconds
        
        for (int i = 0; i <= numSteps; i++) {
            if (i % 24 == 0) {  // Print every 0.4 seconds
                double time = i * dt;
                
                rapier.rapier_rigid_body_get_position(world, bouncyBall, x, y);
                double x1 = x.getValue(), y1 = y.getValue();
                
                rapier.rapier_rigid_body_get_position(world, box, x, y);
                double x2 = x.getValue(), y2 = y.getValue();
                
                rapier.rapier_rigid_body_get_position(world, pushedBall, x, y);
                double x3 = x.getValue(), y3 = y.getValue();
                
                System.out.printf("%6.2fs | (%5.2f,%5.2f) | (%5.2f,%5.2f) | (%5.2f,%5.2f)%n",
                    time, x1, y1, x2, y2, x3, y3);
            }
            
            rapier.rapier_world_step(world);
        }
        
        System.out.println("\n=== All objects have settled ===");
        
        // Clean up
        rapier.rapier_world_destroy(world);
    }
}
