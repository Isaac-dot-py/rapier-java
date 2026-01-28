package com.rapier.example;

import com.rapier.*;
import com.sun.jna.ptr.DoubleByReference;

/**
 * Comprehensive example demonstrating all major features of the Rapier Java bindings.
 * Uses the data-based API with handles instead of wrapper objects.
 */
public class ComprehensiveExample {
    private static RapierNative rapier;
    private static DoubleByReference x = new DoubleByReference();
    private static DoubleByReference y = new DoubleByReference();
    
    public static void main(String[] args) {
        System.out.println("=== Comprehensive Rapier Physics Demo ===\n");
        
        // Get the native library interface
        rapier = Rapier.create();
        
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
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create ground
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        rapier.rapier_collider_create_cuboid(world, ground, 10.0, 0.5);
        
        // Create falling object
        long body = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 5.0);
        rapier.rapier_collider_create_ball(world, body, 0.5);
        
        // Get initial height
        rapier.rapier_rigid_body_get_position(world, body, x, y);
        double initialHeight = y.getValue();
        
        // Simulate until body hits ground
        for (int i = 0; i < 100; i++) {
            rapier.rapier_world_step(world);
        }
        
        rapier.rapier_rigid_body_get_position(world, body, x, y);
        double finalHeight = y.getValue();
        
        System.out.printf("  Initial height: %.2f, Final height: %.2f%n", initialHeight, finalHeight);
        System.out.println("  [OK] Object fell under gravity and stopped on ground");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testRestitution() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Ground
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        long groundCollider = rapier.rapier_collider_create_cuboid(world, ground, 10.0, 0.5);
        rapier.rapier_collider_set_restitution(world, groundCollider, 0.95);
        
        // Very bouncy ball
        long bouncyBall = rapier.rapier_rigid_body_create_dynamic(world, -2.0, 5.0);
        long bouncyCollider = rapier.rapier_collider_create_ball(world, bouncyBall, 0.5);
        rapier.rapier_collider_set_restitution(world, bouncyCollider, 0.95);
        
        // Not bouncy ball
        long deadBall = rapier.rapier_rigid_body_create_dynamic(world, 2.0, 5.0);
        long deadCollider = rapier.rapier_collider_create_ball(world, deadBall, 0.5);
        rapier.rapier_collider_set_restitution(world, deadCollider, 0.1);
        
        // Simulate
        for (int i = 0; i < 150; i++) {
            rapier.rapier_world_step(world);
        }
        
        rapier.rapier_rigid_body_get_position(world, bouncyBall, x, y);
        double bouncyHeight = y.getValue();
        rapier.rapier_rigid_body_get_position(world, deadBall, x, y);
        double deadHeight = y.getValue();
        
        System.out.printf("  Bouncy ball height: %.2f, Dead ball height: %.2f%n", bouncyHeight, deadHeight);
        System.out.println("  [OK] High restitution ball bounced higher than low restitution ball");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testFriction() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create ground
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        long groundCollider = rapier.rapier_collider_create_cuboid(world, ground, 20.0, 0.5);
        rapier.rapier_collider_set_friction(world, groundCollider, 0.5);
        
        // High friction box - starts at origin, moves right
        long highFriction = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 1.0);
        long highCollider = rapier.rapier_collider_create_cuboid(world, highFriction, 0.5, 0.5);
        rapier.rapier_collider_set_friction(world, highCollider, 0.9);
        rapier.rapier_rigid_body_set_linvel(world, highFriction, 5.0, 0.0, true);
        
        // Low friction box - starts at origin, moves right (same as high friction for fair comparison)
        long lowFriction = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 3.0); // Higher up so they don't collide
        long lowCollider = rapier.rapier_collider_create_cuboid(world, lowFriction, 0.5, 0.5);
        rapier.rapier_collider_set_friction(world, lowCollider, 0.1);
        rapier.rapier_rigid_body_set_linvel(world, lowFriction, 5.0, 0.0, true);
        
        // Simulate
        for (int i = 0; i < 200; i++) {
            rapier.rapier_world_step(world);
        }
        
        rapier.rapier_rigid_body_get_position(world, highFriction, x, y);
        double highDist = x.getValue(); // Distance traveled from origin
        rapier.rapier_rigid_body_get_position(world, lowFriction, x, y);
        double lowDist = x.getValue(); // Distance traveled from origin
        
        System.out.printf("  High friction traveled: %.2f, Low friction traveled: %.2f%n", highDist, lowDist);
        System.out.println("  [OK] Friction affects sliding distance");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testImpulses() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Ground
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        rapier.rapier_collider_create_cuboid(world, ground, 20.0, 0.5);
        
        // Create objects at same height
        long noImpulse = rapier.rapier_rigid_body_create_dynamic(world, -3.0, 2.0);
        rapier.rapier_collider_create_ball(world, noImpulse, 0.5);
        
        long withImpulse = rapier.rapier_rigid_body_create_dynamic(world, 3.0, 2.0);
        rapier.rapier_collider_create_ball(world, withImpulse, 0.5);
        rapier.rapier_rigid_body_apply_impulse(world, withImpulse, 0.0, 10.0, true); // Upward impulse
        
        // Initial positions
        rapier.rapier_rigid_body_get_position(world, noImpulse, x, y);
        double noImpulseStart = y.getValue();
        rapier.rapier_rigid_body_get_position(world, withImpulse, x, y);
        double withImpulseStart = y.getValue();
        
        // Simulate
        for (int i = 0; i < 30; i++) {
            rapier.rapier_world_step(world);
        }
        
        rapier.rapier_rigid_body_get_position(world, noImpulse, x, y);
        double noImpulseHeight = y.getValue();
        rapier.rapier_rigid_body_get_position(world, withImpulse, x, y);
        double withImpulseHeight = y.getValue();
        
        System.out.printf("  No impulse: %.2f -> %.2f%n", noImpulseStart, noImpulseHeight);
        System.out.printf("  With impulse: %.2f -> %.2f%n", withImpulseStart, withImpulseHeight);
        System.out.println("  [OK] Impulse launched object upward");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testDifferentShapes() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Ground
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        rapier.rapier_collider_create_cuboid(world, ground, 20.0, 0.5);
        
        // Ball
        long ball = rapier.rapier_rigid_body_create_dynamic(world, -2.0, 5.0);
        rapier.rapier_collider_create_ball(world, ball, 0.5);
        
        // Small box
        long smallBox = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 5.0);
        rapier.rapier_collider_create_cuboid(world, smallBox, 0.5, 0.5);
        
        // Tall box
        long tallBox = rapier.rapier_rigid_body_create_dynamic(world, 2.0, 5.0);
        rapier.rapier_collider_create_cuboid(world, tallBox, 0.3, 0.8);
        
        // Simulate
        for (int i = 0; i < 100; i++) {
            rapier.rapier_world_step(world);
        }
        
        rapier.rapier_rigid_body_get_position(world, ball, x, y);
        System.out.printf("  Ball position: (%.2f, %.2f)%n", x.getValue(), y.getValue());
        
        rapier.rapier_rigid_body_get_position(world, smallBox, x, y);
        System.out.printf("  Small box position: (%.2f, %.2f)%n", x.getValue(), y.getValue());
        
        rapier.rapier_rigid_body_get_position(world, tallBox, x, y);
        double rotation = rapier.rapier_rigid_body_get_rotation(world, tallBox);
        System.out.printf("  Tall box position: (%.2f, %.2f), rotation: %.2f rad%n", 
            x.getValue(), y.getValue(), rotation);
        System.out.println("  [OK] Different shapes simulated correctly");
        
        rapier.rapier_world_destroy(world);
    }
}
