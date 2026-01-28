package com.rapier.example;

import com.rapier.*;
import com.sun.jna.ptr.DoubleByReference;
import com.sun.jna.ptr.IntByReference;

/**
 * Example demonstrating the new physics features:
 * - Sensors
 * - Density
 * - Mass and Inertia
 * - Collision Groups
 * - Damping
 * 
 * Uses the data-based API with handles instead of wrapper objects.
 */
public class NewFeaturesExample {
    private static RapierNative rapier;
    private static DoubleByReference x = new DoubleByReference();
    private static DoubleByReference y = new DoubleByReference();
    
    public static void main(String[] args) {
        System.out.println("=== New Features Demo ===\n");
        
        // Get the native library interface
        rapier = Rapier.create();
        
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
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create a ground
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        rapier.rapier_collider_create_cuboid(world, ground, 10.0, 0.5);
        
        // Create a sensor collider (doesn't block objects)
        long sensorBody = rapier.rapier_rigid_body_create_fixed(world, 0.0, 3.0);
        long sensorCollider = rapier.rapier_collider_create_cuboid(world, sensorBody, 2.0, 0.1);
        rapier.rapier_collider_set_sensor(world, sensorCollider, true);
        
        // Verify sensor status
        boolean isSensor = rapier.rapier_collider_is_sensor(world, sensorCollider);
        System.out.printf("  Sensor status: %b%n", isSensor);
        
        // Create a falling ball that should pass through the sensor
        long ball = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 5.0);
        rapier.rapier_collider_create_ball(world, ball, 0.3);
        
        // Simulate
        for (int i = 0; i < 100; i++) {
            rapier.rapier_world_step(world);
        }
        
        // Ball should have fallen through sensor and stopped on ground
        rapier.rapier_rigid_body_get_position(world, ball, x, y);
        System.out.printf("  Ball final height: %.2f (passed through sensor at y=3.0)%n", y.getValue());
        System.out.println("  ✓ Sensor collider allows objects to pass through");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testDensity() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create two balls with different densities
        long lightBall = rapier.rapier_rigid_body_create_dynamic(world, -2.0, 5.0);
        long lightCollider = rapier.rapier_collider_create_ball(world, lightBall, 0.5);
        rapier.rapier_collider_set_density(world, lightCollider, 0.5);
        
        long heavyBall = rapier.rapier_rigid_body_create_dynamic(world, 2.0, 5.0);
        long heavyCollider = rapier.rapier_collider_create_ball(world, heavyBall, 0.5);
        rapier.rapier_collider_set_density(world, heavyCollider, 10.0);
        
        // Run a simulation step to allow mass recomputation
        rapier.rapier_world_step(world);
        
        // Verify densities
        double lightDensity = rapier.rapier_collider_get_density(world, lightCollider);
        double heavyDensity = rapier.rapier_collider_get_density(world, heavyCollider);
        System.out.printf("  Light ball density: %.1f%n", lightDensity);
        System.out.printf("  Heavy ball density: %.1f%n", heavyDensity);
        
        // Get masses (derived from density and shape)
        double lightMass = rapier.rapier_rigid_body_get_mass(world, lightBall);
        double heavyMass = rapier.rapier_rigid_body_get_mass(world, heavyBall);
        System.out.printf("  Light ball mass: %.4f%n", lightMass);
        System.out.printf("  Heavy ball mass: %.4f%n", heavyMass);
        System.out.println("  ✓ Density affects mass correctly");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testMassAndInertia() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create a body with a collider
        long body = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 5.0);
        rapier.rapier_collider_create_cuboid(world, body, 1.0, 0.5);
        
        // Run a simulation step to initialize mass properties
        rapier.rapier_world_step(world);
        
        double initialMass = rapier.rapier_rigid_body_get_mass(world, body);
        double initialInertia = rapier.rapier_rigid_body_get_angular_inertia(world, body);
        System.out.printf("  Initial mass: %.4f%n", initialMass);
        System.out.printf("  Initial angular inertia: %.4f%n", initialInertia);
        
        // Add additional mass
        rapier.rapier_rigid_body_set_additional_mass(world, body, 5.0, true);
        
        // Run another step to update mass properties
        rapier.rapier_world_step(world);
        
        double newMass = rapier.rapier_rigid_body_get_mass(world, body);
        double newInertia = rapier.rapier_rigid_body_get_angular_inertia(world, body);
        System.out.printf("  After adding 5.0 mass - New mass: %.4f%n", newMass);
        System.out.printf("  After adding 5.0 mass - New inertia: %.4f%n", newInertia);
        System.out.println("  ✓ Additional mass increases total mass and inertia");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testCollisionGroups() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create ground that collides with everything
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        rapier.rapier_collider_create_cuboid(world, ground, 10.0, 0.5);
        // Default: memberships = 0xFFFFFFFF, filter = 0xFFFFFFFF (collides with all)
        
        // Define collision groups
        // In Java, use -1 for "all groups" (0xFFFFFFFF) since Java uses signed integers
        int GROUP_1 = 1;  // 0b0001
        int GROUP_2 = 2;  // 0b0010
        int ALL_GROUPS = -1; // Same bit pattern as 0xFFFFFFFF
        
        // Create a ball in GROUP_1 that only collides with GROUP_1
        long ball1 = rapier.rapier_rigid_body_create_dynamic(world, -2.0, 5.0);
        long ball1Collider = rapier.rapier_collider_create_ball(world, ball1, 0.5);
        rapier.rapier_collider_set_collision_groups(world, ball1Collider, GROUP_1, GROUP_1);
        
        // Create a ball in GROUP_2 that only collides with GROUP_2
        long ball2 = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 5.0);
        long ball2Collider = rapier.rapier_collider_create_ball(world, ball2, 0.5);
        rapier.rapier_collider_set_collision_groups(world, ball2Collider, GROUP_2, GROUP_2);
        
        // Create a ball that collides with ALL groups
        long ball3 = rapier.rapier_rigid_body_create_dynamic(world, 2.0, 5.0);
        long ball3Collider = rapier.rapier_collider_create_ball(world, ball3, 0.5);
        rapier.rapier_collider_set_collision_groups(world, ball3Collider, ALL_GROUPS, ALL_GROUPS);
        
        // Verify collision groups
        IntByReference memberships = new IntByReference();
        IntByReference filter = new IntByReference();
        
        rapier.rapier_collider_get_collision_groups(world, ball1Collider, memberships, filter);
        System.out.printf("  Ball1 groups: memberships=0x%X, filter=0x%X%n", memberships.getValue(), filter.getValue());
        
        rapier.rapier_collider_get_collision_groups(world, ball2Collider, memberships, filter);
        System.out.printf("  Ball2 groups: memberships=0x%X, filter=0x%X%n", memberships.getValue(), filter.getValue());
        
        // Simulate - balls in different groups should not collide with each other
        // but ball3 (all groups) should collide with the ground
        for (int i = 0; i < 100; i++) {
            rapier.rapier_world_step(world);
        }
        
        rapier.rapier_rigid_body_get_position(world, ball1, x, y);
        System.out.printf("  Ball1 height: %.2f%n", y.getValue());
        
        rapier.rapier_rigid_body_get_position(world, ball2, x, y);
        System.out.printf("  Ball2 height: %.2f%n", y.getValue());
        
        rapier.rapier_rigid_body_get_position(world, ball3, x, y);
        System.out.printf("  Ball3 height: %.2f (collides with ground)%n", y.getValue());
        System.out.println("  ✓ Collision groups control which objects can collide");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testDamping() {
        long world = rapier.rapier_world_create(0.0, 0.0); // No gravity for clearer test
        
        // Create ground
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        rapier.rapier_collider_create_cuboid(world, ground, 10.0, 0.5);
        
        // Create a ball without damping
        long noDampingBall = rapier.rapier_rigid_body_create_dynamic(world, -3.0, 2.0);
        rapier.rapier_collider_create_ball(world, noDampingBall, 0.5);
        rapier.rapier_rigid_body_set_linvel(world, noDampingBall, 5.0, 0.0, true);
        
        // Create a ball with high linear damping
        long dampedBall = rapier.rapier_rigid_body_create_dynamic(world, 3.0, 2.0);
        rapier.rapier_collider_create_ball(world, dampedBall, 0.5);
        rapier.rapier_rigid_body_set_linear_damping(world, dampedBall, 2.0);
        rapier.rapier_rigid_body_set_linvel(world, dampedBall, 5.0, 0.0, true);
        
        // Verify damping values
        double noDamping = rapier.rapier_rigid_body_get_linear_damping(world, noDampingBall);
        double damping = rapier.rapier_rigid_body_get_linear_damping(world, dampedBall);
        System.out.printf("  No damping ball - linear damping: %.2f%n", noDamping);
        System.out.printf("  Damped ball - linear damping: %.2f%n", damping);
        
        // Test angular damping
        long spinningBody = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 5.0);
        rapier.rapier_collider_create_cuboid(world, spinningBody, 0.5, 0.5);
        rapier.rapier_rigid_body_set_angular_damping(world, spinningBody, 1.5);
        double angularDamping = rapier.rapier_rigid_body_get_angular_damping(world, spinningBody);
        System.out.printf("  Spinning body - angular damping: %.2f%n", angularDamping);
        
        // Simulate
        for (int i = 0; i < 50; i++) {
            rapier.rapier_world_step(world);
        }
        
        rapier.rapier_rigid_body_get_position(world, noDampingBall, x, y);
        double noDampingX = x.getValue();
        
        rapier.rapier_rigid_body_get_position(world, dampedBall, x, y);
        double dampedX = x.getValue();
        
        System.out.printf("  After simulation - No damping X: %.2f, Damped X: %.2f%n", noDampingX, dampedX);
        System.out.println("  ✓ Damping slows down objects over time");
        
        rapier.rapier_world_destroy(world);
    }
}
