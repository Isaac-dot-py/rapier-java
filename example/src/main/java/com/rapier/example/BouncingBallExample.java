package com.rapier.example;

import com.rapier.*;

/**
 * Example demonstrating a simple bouncing ball simulation using Rapier.
 */
public class BouncingBallExample {
    public static void main(String[] args) {
        System.out.println("=== Rapier Java 2D Physics Example ===");
        System.out.println("Simulating a bouncing ball...\n");
        
        // Create a physics world with gravity
        PhysicsWorld world = new PhysicsWorld(0.0, -9.81);
        
        // Create a ground (fixed body)
        RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
        Collider groundCollider = world.createCuboidCollider(ground, 50.0, 1.0);
        groundCollider.setFriction(0.5);
        groundCollider.setRestitution(0.0);
        
        // Create a ball (dynamic body) at height 10.0
        RigidBody ball = world.createDynamicRigidBody(0.0, 10.0);
        Collider ballCollider = world.createBallCollider(ball, 0.5);
        ballCollider.setRestitution(0.8);  // 80% bounce
        ballCollider.setFriction(0.3);
        
        // Simulate for 5 seconds (at 60 FPS = 300 steps)
        double dt = 1.0 / 60.0;
        int numSteps = 300;
        
        System.out.println("Time(s)   | Position(x, y)        | Height");
        System.out.println("----------|----------------------|--------");
        
        for (int i = 0; i <= numSteps; i++) {
            // Print state every 30 steps (0.5 seconds)
            if (i % 30 == 0) {
                Vector2 pos = ball.getPosition();
                double time = i * dt;
                System.out.printf("%7.2f   | %8.3f, %8.3f  | %6.3fm%n", 
                    time, pos.x, pos.y, pos.y);
            }
            
            // Step the simulation
            world.step();
        }
        
        // Final position
        Vector2 finalPos = ball.getPosition();
        System.out.println("\nSimulation complete!");
        System.out.printf("Final ball position: (%.3f, %.3f)%n", finalPos.x, finalPos.y);
        System.out.printf("Final rotation: %.3f radians%n", ball.getRotation());
        
        // Clean up
        world.destroy();
        
        System.out.println("\n=== Example completed successfully ===");
    }
}
