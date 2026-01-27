package com.rapier.example;

import com.rapier.*;

/**
 * Example demonstrating multiple objects with different properties.
 */
public class MultiObjectExample {
    public static void main(String[] args) {
        System.out.println("=== Rapier Multi-Object Example ===");
        System.out.println("Simulating multiple falling objects...\n");
        
        // Create a physics world
        PhysicsWorld world = new PhysicsWorld(9.81);
        
        // Create ground
        RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
        Collider groundCollider = world.createCuboidCollider(ground, 100.0, 1.0);
        groundCollider.setRestitution(0.2);
        
        // Create a bouncy ball
        RigidBody bouncyBall = world.createDynamicRigidBody(-5.0, 15.0);
        Collider bouncyCollider = world.createBallCollider(bouncyBall, 0.5);
        bouncyCollider.setRestitution(0.9);  // Very bouncy
        bouncyCollider.setFriction(0.1);
        
        // Create a heavy box
        RigidBody box = world.createDynamicRigidBody(0.0, 12.0);
        Collider boxCollider = world.createCuboidCollider(box, 0.7, 0.7);
        boxCollider.setRestitution(0.3);  // Less bouncy
        boxCollider.setFriction(0.5);
        
        // Create another ball with impulse
        RigidBody pushedBall = world.createDynamicRigidBody(5.0, 10.0);
        Collider pushedCollider = world.createBallCollider(pushedBall, 0.4);
        pushedCollider.setRestitution(0.6);
        pushedCollider.setFriction(0.2);
        pushedBall.applyImpulse(-3.0, 2.0, true);  // Give it a push
        
        // Simulate
        System.out.println("Time(s) | Bouncy Ball   | Box           | Pushed Ball");
        System.out.println("--------|---------------|---------------|---------------");
        
        double dt = 1.0 / 60.0;
        int numSteps = 240;  // 4 seconds
        
        for (int i = 0; i <= numSteps; i++) {
            if (i % 24 == 0) {  // Print every 0.4 seconds
                double time = i * dt;
                Vector2 pos1 = bouncyBall.getPosition();
                Vector2 pos2 = box.getPosition();
                Vector2 pos3 = pushedBall.getPosition();
                
                System.out.printf("%6.2fs | (%5.2f,%5.2f) | (%5.2f,%5.2f) | (%5.2f,%5.2f)%n",
                    time, pos1.x, pos1.y, pos2.x, pos2.y, pos3.x, pos3.y);
            }
            
            world.step();
        }
        
        System.out.println("\n=== All objects have settled ===");
        
        // Clean up
        world.destroy();
    }
}
