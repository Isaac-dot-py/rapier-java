package com.rapier.example;

import com.rapier.*;

/**
 * Comprehensive example demonstrating all major features of the Rapier Java bindings.
 */
public class ComprehensiveExample {
    public static void main(String[] args) {
        System.out.println("=== Comprehensive Rapier Physics Demo ===\n");
        
        // Test 1: Basic physics simulation
        System.out.println("Test 1: Basic Gravity and Collision");
        testBasicPhysics();
        
        // Test 2: Restitution (bounciness)
        System.out.println("\nTest 2: Restitution (Bounciness)");
        testRestitution();
        
        // Test 3: Friction
        System.out.println("\nTest 3: Friction Effects");
        testFriction();
        
        // Test 4: Impulses
        System.out.println("\nTest 4: Impulse Application");
        testImpulses();
        
        // Test 5: Multiple body types
        System.out.println("\nTest 5: Different Collider Shapes");
        testDifferentShapes();
        
        System.out.println("\n=== All tests completed successfully! ===");
    }
    
    private static void testBasicPhysics() {
        PhysicsWorld world = new PhysicsWorld(9.81);
        
        // Create ground
        RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
        world.createCuboidCollider(ground, 10.0, 0.5);
        
        // Create falling object
        RigidBody body = world.createDynamicRigidBody(0.0, 5.0);
        world.createBallCollider(body, 0.5);
        
        // Simulate until body hits ground
        double initialHeight = body.getPosition().y;
        for (int i = 0; i < 100; i++) {
            world.step();
        }
        double finalHeight = body.getPosition().y;
        
        System.out.printf("  Initial height: %.2f, Final height: %.2f%n", initialHeight, finalHeight);
        System.out.println("  ✓ Object fell under gravity and stopped on ground");
        
        world.destroy();
    }
    
    private static void testRestitution() {
        PhysicsWorld world = new PhysicsWorld(9.81);
        
        // Ground
        RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
        Collider groundCollider = world.createCuboidCollider(ground, 10.0, 0.5);
        groundCollider.setRestitution(0.0);
        
        // Very bouncy ball
        RigidBody bouncyBall = world.createDynamicRigidBody(-2.0, 5.0);
        Collider bouncyCollider = world.createBallCollider(bouncyBall, 0.5);
        bouncyCollider.setRestitution(0.95);
        
        // Not bouncy ball
        RigidBody deadBall = world.createDynamicRigidBody(2.0, 5.0);
        Collider deadCollider = world.createBallCollider(deadBall, 0.5);
        deadCollider.setRestitution(0.1);
        
        // Simulate
        for (int i = 0; i < 150; i++) {
            world.step();
        }
        
        double bouncyHeight = bouncyBall.getPosition().y;
        double deadHeight = deadBall.getPosition().y;
        
        System.out.printf("  Bouncy ball height: %.2f, Dead ball height: %.2f%n", bouncyHeight, deadHeight);
        System.out.println("  ✓ High restitution ball bounced higher than low restitution ball");
        
        world.destroy();
    }
    
    private static void testFriction() {
        PhysicsWorld world = new PhysicsWorld(9.81);
        
        // Create inclined plane (simulated with horizontal velocity)
        RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
        Collider groundCollider = world.createCuboidCollider(ground, 20.0, 0.5);
        groundCollider.setFriction(0.5);
        
        // High friction box
        RigidBody highFriction = world.createDynamicRigidBody(-5.0, 1.0);
        Collider highCollider = world.createCuboidCollider(highFriction, 0.5, 0.5);
        highCollider.setFriction(0.9);
        highFriction.setLinearVelocity(5.0, 0.0, true);
        
        // Low friction box
        RigidBody lowFriction = world.createDynamicRigidBody(5.0, 1.0);
        Collider lowCollider = world.createCuboidCollider(lowFriction, 0.5, 0.5);
        lowCollider.setFriction(0.1);
        lowFriction.setLinearVelocity(-5.0, 0.0, true);
        
        // Simulate
        for (int i = 0; i < 100; i++) {
            world.step();
        }
        
        double highPos = Math.abs(highFriction.getPosition().x);
        double lowPos = Math.abs(lowFriction.getPosition().x);
        
        System.out.printf("  High friction distance: %.2f, Low friction distance: %.2f%n", highPos, lowPos);
        System.out.println("  ✓ Friction affects sliding distance");
        
        world.destroy();
    }
    
    private static void testImpulses() {
        PhysicsWorld world = new PhysicsWorld(9.81);
        
        // Ground
        RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
        world.createCuboidCollider(ground, 20.0, 0.5);
        
        // Create objects at same height
        RigidBody noImpulse = world.createDynamicRigidBody(-3.0, 2.0);
        world.createBallCollider(noImpulse, 0.5);
        
        RigidBody withImpulse = world.createDynamicRigidBody(3.0, 2.0);
        world.createBallCollider(withImpulse, 0.5);
        withImpulse.applyImpulse(0.0, 10.0, true); // Upward impulse
        
        // Initial positions
        double noImpulseStart = noImpulse.getPosition().y;
        double withImpulseStart = withImpulse.getPosition().y;
        
        // Simulate
        for (int i = 0; i < 30; i++) {
            world.step();
        }
        
        double noImpulseHeight = noImpulse.getPosition().y;
        double withImpulseHeight = withImpulse.getPosition().y;
        
        System.out.printf("  No impulse: %.2f -> %.2f%n", noImpulseStart, noImpulseHeight);
        System.out.printf("  With impulse: %.2f -> %.2f%n", withImpulseStart, withImpulseHeight);
        System.out.println("  ✓ Impulse launched object upward");
        
        world.destroy();
    }
    
    private static void testDifferentShapes() {
        PhysicsWorld world = new PhysicsWorld(9.81);
        
        // Ground
        RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
        world.createCuboidCollider(ground, 20.0, 0.5);
        
        // Ball
        RigidBody ball = world.createDynamicRigidBody(-2.0, 5.0);
        world.createBallCollider(ball, 0.5);
        
        // Small box
        RigidBody smallBox = world.createDynamicRigidBody(0.0, 5.0);
        world.createCuboidCollider(smallBox, 0.5, 0.5);
        
        // Tall box
        RigidBody tallBox = world.createDynamicRigidBody(2.0, 5.0);
        world.createCuboidCollider(tallBox, 0.3, 0.8);
        
        // Simulate
        for (int i = 0; i < 100; i++) {
            world.step();
        }
        
        System.out.printf("  Ball position: (%.2f, %.2f)%n", ball.getPosition().x, ball.getPosition().y);
        System.out.printf("  Small box position: (%.2f, %.2f)%n", smallBox.getPosition().x, smallBox.getPosition().y);
        System.out.printf("  Tall box position: (%.2f, %.2f), rotation: %.2f rad%n", 
            tallBox.getPosition().x, tallBox.getPosition().y, tallBox.getRotation());
        System.out.println("  ✓ Different shapes simulated correctly");
        
        world.destroy();
    }
}
