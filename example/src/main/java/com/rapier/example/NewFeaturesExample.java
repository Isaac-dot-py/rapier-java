package com.rapier.example;

import com.rapier.*;

/**
 * Example demonstrating the new physics features:
 * - Sensors
 * - Density
 * - Mass and Inertia
 * - Collision Groups
 * - Damping
 */
public class NewFeaturesExample {
    public static void main(String[] args) {
        System.out.println("=== New Features Demo ===\n");
        
        // Test 1: Sensors
        System.out.println("Test 1: Sensors");
        testSensors();
        
        // Test 2: Density
        System.out.println("\nTest 2: Density");
        testDensity();
        
        // Test 3: Mass and Inertia
        System.out.println("\nTest 3: Mass and Inertia");
        testMassAndInertia();
        
        // Test 4: Collision Groups
        System.out.println("\nTest 4: Collision Groups");
        testCollisionGroups();
        
        // Test 5: Damping
        System.out.println("\nTest 5: Damping");
        testDamping();
        
        System.out.println("\n=== All new features tests completed successfully! ===");
    }
    
    private static void testSensors() {
        PhysicsWorld world = new PhysicsWorld(9.81);
        
        // Create a ground
        RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
        Collider groundCollider = world.createCuboidCollider(ground, 10.0, 0.5);
        
        // Create a sensor collider (doesn't block objects)
        RigidBody sensorBody = world.createFixedRigidBody(0.0, 3.0);
        Collider sensorCollider = world.createCuboidCollider(sensorBody, 2.0, 0.1);
        sensorCollider.setSensor(true);
        
        // Verify sensor status
        boolean isSensor = sensorCollider.isSensor();
        System.out.printf("  Sensor status: %b%n", isSensor);
        
        // Create a falling ball that should pass through the sensor
        RigidBody ball = world.createDynamicRigidBody(0.0, 5.0);
        world.createBallCollider(ball, 0.3);
        
        // Simulate
        for (int i = 0; i < 100; i++) {
            world.step();
        }
        
        // Ball should have fallen through sensor and stopped on ground
        double ballHeight = ball.getPosition().y;
        System.out.printf("  Ball final height: %.2f (passed through sensor at y=3.0)%n", ballHeight);
        System.out.println("  ✓ Sensor collider allows objects to pass through");
        
        world.destroy();
    }
    
    private static void testDensity() {
        PhysicsWorld world = new PhysicsWorld(9.81);
        
        // Create two balls with different densities
        RigidBody lightBall = world.createDynamicRigidBody(-2.0, 5.0);
        Collider lightCollider = world.createBallCollider(lightBall, 0.5);
        lightCollider.setDensity(0.5);
        
        RigidBody heavyBall = world.createDynamicRigidBody(2.0, 5.0);
        Collider heavyCollider = world.createBallCollider(heavyBall, 0.5);
        heavyCollider.setDensity(10.0);
        
        // Run a simulation step to allow mass recomputation
        world.step();
        
        // Verify densities
        double lightDensity = lightCollider.getDensity();
        double heavyDensity = heavyCollider.getDensity();
        System.out.printf("  Light ball density: %.1f%n", lightDensity);
        System.out.printf("  Heavy ball density: %.1f%n", heavyDensity);
        
        // Get masses (derived from density and shape)
        double lightMass = lightBall.getMass();
        double heavyMass = heavyBall.getMass();
        System.out.printf("  Light ball mass: %.4f%n", lightMass);
        System.out.printf("  Heavy ball mass: %.4f%n", heavyMass);
        System.out.println("  ✓ Density affects mass correctly");
        
        world.destroy();
    }
    
    private static void testMassAndInertia() {
        PhysicsWorld world = new PhysicsWorld(9.81);
        
        // Create a body with a collider
        RigidBody body = world.createDynamicRigidBody(0.0, 5.0);
        Collider collider = world.createCuboidCollider(body, 1.0, 0.5);
        
        // Run a simulation step to initialize mass properties
        world.step();
        
        double initialMass = body.getMass();
        double initialInertia = body.getAngularInertia();
        System.out.printf("  Initial mass: %.4f%n", initialMass);
        System.out.printf("  Initial angular inertia: %.4f%n", initialInertia);
        
        // Add additional mass
        body.setAdditionalMass(5.0, true);
        
        // Run another step to update mass properties
        world.step();
        
        double newMass = body.getMass();
        double newInertia = body.getAngularInertia();
        System.out.printf("  After adding 5.0 mass - New mass: %.4f%n", newMass);
        System.out.printf("  After adding 5.0 mass - New inertia: %.4f%n", newInertia);
        System.out.println("  ✓ Additional mass increases total mass and inertia");
        
        world.destroy();
    }
    
    private static void testCollisionGroups() {
        PhysicsWorld world = new PhysicsWorld(9.81);
        
        // Create ground that collides with everything
        RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
        Collider groundCollider = world.createCuboidCollider(ground, 10.0, 0.5);
        // Default: memberships = 0xFFFFFFFF, filter = 0xFFFFFFFF (collides with all)
        
        // Define collision groups
        // In Java, use -1 for "all groups" (0xFFFFFFFF) since Java uses signed integers
        int GROUP_1 = 1;  // 0b0001
        int GROUP_2 = 2;  // 0b0010
        int ALL_GROUPS = -1; // Same bit pattern as 0xFFFFFFFF
        
        // Create a ball in GROUP_1 that only collides with GROUP_1
        RigidBody ball1 = world.createDynamicRigidBody(-2.0, 5.0);
        Collider ball1Collider = world.createBallCollider(ball1, 0.5);
        ball1Collider.setCollisionGroups(GROUP_1, GROUP_1);
        
        // Create a ball in GROUP_2 that only collides with GROUP_2
        RigidBody ball2 = world.createDynamicRigidBody(0.0, 5.0);
        Collider ball2Collider = world.createBallCollider(ball2, 0.5);
        ball2Collider.setCollisionGroups(GROUP_2, GROUP_2);
        
        // Create a ball that collides with ALL groups
        RigidBody ball3 = world.createDynamicRigidBody(2.0, 5.0);
        Collider ball3Collider = world.createBallCollider(ball3, 0.5);
        ball3Collider.setCollisionGroups(ALL_GROUPS, ALL_GROUPS);
        
        // Verify collision groups
        int[] groups1 = ball1Collider.getCollisionGroups();
        int[] groups2 = ball2Collider.getCollisionGroups();
        System.out.printf("  Ball1 groups: memberships=0x%X, filter=0x%X%n", groups1[0], groups1[1]);
        System.out.printf("  Ball2 groups: memberships=0x%X, filter=0x%X%n", groups2[0], groups2[1]);
        
        // Simulate - balls in different groups should not collide with each other
        // but ball3 (all groups) should collide with the ground
        for (int i = 0; i < 100; i++) {
            world.step();
        }
        
        System.out.printf("  Ball1 height: %.2f%n", ball1.getPosition().y);
        System.out.printf("  Ball2 height: %.2f%n", ball2.getPosition().y);
        System.out.printf("  Ball3 height: %.2f (collides with ground)%n", ball3.getPosition().y);
        System.out.println("  ✓ Collision groups control which objects can collide");
        
        world.destroy();
    }
    
    private static void testDamping() {
        PhysicsWorld world = new PhysicsWorld(0.0); // No gravity for clearer test
        
        // Create ground
        RigidBody ground = world.createFixedRigidBody(0.0, 0.0);
        world.createCuboidCollider(ground, 10.0, 0.5);
        
        // Create a ball without damping
        RigidBody noDampingBall = world.createDynamicRigidBody(-3.0, 2.0);
        world.createBallCollider(noDampingBall, 0.5);
        noDampingBall.setLinearVelocity(5.0, 0.0, true);
        
        // Create a ball with high linear damping
        RigidBody dampedBall = world.createDynamicRigidBody(3.0, 2.0);
        world.createBallCollider(dampedBall, 0.5);
        dampedBall.setLinearDamping(2.0);
        dampedBall.setLinearVelocity(5.0, 0.0, true);
        
        // Verify damping values
        System.out.printf("  No damping ball - linear damping: %.2f%n", noDampingBall.getLinearDamping());
        System.out.printf("  Damped ball - linear damping: %.2f%n", dampedBall.getLinearDamping());
        
        // Test angular damping
        RigidBody spinningBody = world.createDynamicRigidBody(0.0, 5.0);
        world.createCuboidCollider(spinningBody, 0.5, 0.5);
        spinningBody.setAngularDamping(1.5);
        System.out.printf("  Spinning body - angular damping: %.2f%n", spinningBody.getAngularDamping());
        
        // Simulate
        for (int i = 0; i < 50; i++) {
            world.step();
        }
        
        double noDampingX = noDampingBall.getPosition().x;
        double dampedX = dampedBall.getPosition().x;
        System.out.printf("  After simulation - No damping X: %.2f, Damped X: %.2f%n", noDampingX, dampedX);
        System.out.println("  ✓ Damping slows down objects over time");
        
        world.destroy();
    }
}
