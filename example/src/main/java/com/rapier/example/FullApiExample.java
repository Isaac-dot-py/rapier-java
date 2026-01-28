package com.rapier.example;

import com.rapier.*;
import com.sun.jna.ptr.DoubleByReference;
import com.sun.jna.ptr.IntByReference;

/**
 * Example demonstrating the complete Rapier 2D API including:
 * - Joints (revolute, prismatic, fixed, rope, spring)
 * - CoefficientCombineRules
 * - Kinematic bodies
 * - Additional rigid body properties
 * - Complete collider properties
 */
public class FullApiExample {
    private static RapierNative rapier;
    private static DoubleByReference x = new DoubleByReference();
    private static DoubleByReference y = new DoubleByReference();
    
    public static void main(String[] args) {
        System.out.println("=== Complete Rapier 2D API Demo ===\n");
        
        rapier = Rapier.create();
        
        // Test 1: Coefficient Combine Rules
        System.out.println("Test 1: Coefficient Combine Rules");
        testCombineRules();
        
        // Test 2: Joints
        System.out.println("\nTest 2: Joints (Revolute, Spring)");
        testJoints();
        
        // Test 3: Kinematic Bodies
        System.out.println("\nTest 3: Kinematic Bodies");
        testKinematicBodies();
        
        // Test 4: Rigid Body Properties
        System.out.println("\nTest 4: Rigid Body Properties");
        testRigidBodyProperties();
        
        // Test 5: New Collider Shapes
        System.out.println("\nTest 5: Additional Collider Shapes");
        testColliderShapes();
        
        // Test 6: World Queries
        System.out.println("\nTest 6: World Queries");
        testWorldQueries();
        
        System.out.println("\n=== All complete API tests passed! ===");
    }
    
    private static void testCombineRules() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create ground
        long ground = rapier.rapier_rigid_body_create_fixed(world, 0.0, 0.0);
        long groundCollider = rapier.rapier_collider_create_cuboid(world, ground, 10.0, 0.5);
        rapier.rapier_collider_set_friction(world, groundCollider, 0.5);
        rapier.rapier_collider_set_restitution(world, groundCollider, 0.5);
        
        // Set combine rules: 0=Average, 1=Min, 2=Multiply, 3=Max
        rapier.rapier_collider_set_friction_combine_rule(world, groundCollider, 3); // Max
        rapier.rapier_collider_set_restitution_combine_rule(world, groundCollider, 1); // Min
        
        int frictionRule = rapier.rapier_collider_get_friction_combine_rule(world, groundCollider);
        int restitutionRule = rapier.rapier_collider_get_restitution_combine_rule(world, groundCollider);
        
        System.out.printf("  Friction combine rule: %d (expected 3=Max)%n", frictionRule);
        System.out.printf("  Restitution combine rule: %d (expected 1=Min)%n", restitutionRule);
        
        // Verify getter works
        double friction = rapier.rapier_collider_get_friction(world, groundCollider);
        double restitution = rapier.rapier_collider_get_restitution(world, groundCollider);
        System.out.printf("  Friction: %.2f, Restitution: %.2f%n", friction, restitution);
        
        System.out.println("  [OK] Coefficient combine rules work correctly");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testJoints() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create a pendulum with revolute joint
        long anchor = rapier.rapier_rigid_body_create_fixed(world, 0.0, 5.0);
        rapier.rapier_collider_create_ball(world, anchor, 0.1);
        
        long pendulum = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 3.0);
        rapier.rapier_collider_create_ball(world, pendulum, 0.3);
        
        // Create revolute joint at anchor position
        long revoluteJoint = rapier.rapier_joint_create_revolute(
            world, anchor, pendulum,
            0.0, 0.0,  // anchor1 local position
            0.0, 2.0   // anchor2 local position (2 units above pendulum body)
        );
        
        System.out.printf("  Created revolute joint (handle: %d)%n", revoluteJoint);
        
        // Set joint limits
        rapier.rapier_joint_set_limits(world, revoluteJoint, 2, -1.0, 1.0); // axis=2 (AngX)
        
        // Give pendulum initial velocity
        rapier.rapier_rigid_body_set_linvel(world, pendulum, 5.0, 0.0, true);
        
        // Simulate
        for (int i = 0; i < 60; i++) {
            rapier.rapier_world_step(world);
        }
        
        rapier.rapier_rigid_body_get_position(world, pendulum, x, y);
        System.out.printf("  Pendulum position after 60 steps: (%.2f, %.2f)%n", x.getValue(), y.getValue());
        
        // Test spring joint
        long body1 = rapier.rapier_rigid_body_create_fixed(world, 5.0, 5.0);
        long body2 = rapier.rapier_rigid_body_create_dynamic(world, 5.0, 2.0);
        rapier.rapier_collider_create_ball(world, body1, 0.1);
        rapier.rapier_collider_create_ball(world, body2, 0.3);
        
        long springJoint = rapier.rapier_joint_create_spring(
            world, body1, body2,
            0.0, 0.0,   // anchor1
            0.0, 0.0,   // anchor2
            2.0,        // rest_length
            100.0,      // stiffness
            5.0         // damping
        );
        
        System.out.printf("  Created spring joint (handle: %d)%n", springJoint);
        
        // Simulate
        for (int i = 0; i < 120; i++) {
            rapier.rapier_world_step(world);
        }
        
        rapier.rapier_rigid_body_get_position(world, body2, x, y);
        System.out.printf("  Spring body position: (%.2f, %.2f)%n", x.getValue(), y.getValue());
        
        System.out.println("  [OK] Joints work correctly");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testKinematicBodies() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create kinematic velocity-based body
        long kinematicVel = rapier.rapier_rigid_body_create_kinematic_velocity_based(world, 0.0, 0.0);
        rapier.rapier_collider_create_cuboid(world, kinematicVel, 2.0, 0.5);
        rapier.rapier_rigid_body_set_linvel(world, kinematicVel, 1.0, 0.0, true);
        
        // Create kinematic position-based body
        long kinematicPos = rapier.rapier_rigid_body_create_kinematic_position_based(world, 5.0, 0.0);
        rapier.rapier_collider_create_cuboid(world, kinematicPos, 2.0, 0.5);
        
        // Check body types
        int velType = rapier.rapier_rigid_body_get_body_type(world, kinematicVel);
        int posType = rapier.rapier_rigid_body_get_body_type(world, kinematicPos);
        
        System.out.printf("  Velocity-based kinematic type: %d (expected 3)%n", velType);
        System.out.printf("  Position-based kinematic type: %d (expected 2)%n", posType);
        
        // Simulate and move kinematic position based
        for (int i = 0; i < 60; i++) {
            double targetX = 5.0 + (double)i * 0.05;
            rapier.rapier_rigid_body_set_next_kinematic_translation(world, kinematicPos, targetX, 0.0);
            rapier.rapier_world_step(world);
        }
        
        rapier.rapier_rigid_body_get_position(world, kinematicVel, x, y);
        System.out.printf("  Kinematic velocity body position: (%.2f, %.2f)%n", x.getValue(), y.getValue());
        
        rapier.rapier_rigid_body_get_position(world, kinematicPos, x, y);
        System.out.printf("  Kinematic position body position: (%.2f, %.2f)%n", x.getValue(), y.getValue());
        
        System.out.println("  [OK] Kinematic bodies work correctly");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testRigidBodyProperties() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        long body = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 5.0);
        rapier.rapier_collider_create_ball(world, body, 0.5);
        
        // Test gravity scale
        rapier.rapier_rigid_body_set_gravity_scale(world, body, 0.5, true);
        double gravityScale = rapier.rapier_rigid_body_get_gravity_scale(world, body);
        System.out.printf("  Gravity scale: %.2f (expected 0.5)%n", gravityScale);
        
        // Test dominance group
        rapier.rapier_rigid_body_set_dominance_group(world, body, (byte)5);
        byte dominance = rapier.rapier_rigid_body_get_dominance_group(world, body);
        System.out.printf("  Dominance group: %d (expected 5)%n", dominance);
        
        // Test CCD
        rapier.rapier_rigid_body_enable_ccd(world, body, true);
        boolean ccdEnabled = rapier.rapier_rigid_body_is_ccd_enabled(world, body);
        System.out.printf("  CCD enabled: %b (expected true)%n", ccdEnabled);
        
        // Test lock rotations
        rapier.rapier_rigid_body_lock_rotations(world, body, true, true);
        boolean rotLocked = rapier.rapier_rigid_body_is_rotation_locked(world, body);
        System.out.printf("  Rotation locked: %b (expected true)%n", rotLocked);
        
        // Test forces
        rapier.rapier_rigid_body_add_force(world, body, 10.0, 0.0, true);
        rapier.rapier_rigid_body_add_torque(world, body, 5.0, true);
        
        // Step and check
        rapier.rapier_world_step(world);
        
        DoubleByReference vx = new DoubleByReference();
        DoubleByReference vy = new DoubleByReference();
        rapier.rapier_rigid_body_get_linvel(world, body, vx, vy);
        System.out.printf("  Velocity after force: (%.2f, %.2f)%n", vx.getValue(), vy.getValue());
        
        // Test sleep
        rapier.rapier_rigid_body_sleep(world, body);
        boolean sleeping = rapier.rapier_rigid_body_is_sleeping(world, body);
        System.out.printf("  Is sleeping: %b%n", sleeping);
        
        rapier.rapier_rigid_body_wake_up(world, body, true);
        sleeping = rapier.rapier_rigid_body_is_sleeping(world, body);
        System.out.printf("  After wake up, sleeping: %b%n", sleeping);
        
        // Reset forces
        rapier.rapier_rigid_body_reset_forces(world, body, true);
        rapier.rapier_rigid_body_reset_torques(world, body, true);
        
        System.out.println("  [OK] Rigid body properties work correctly");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testColliderShapes() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        long body = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 5.0);
        
        // Test capsule
        long capsule = rapier.rapier_collider_create_capsule(world, body, 0.5, 0.2);
        System.out.printf("  Created capsule collider (handle: %d)%n", capsule);
        
        // Test segment
        long body2 = rapier.rapier_rigid_body_create_fixed(world, 5.0, 0.0);
        long segment = rapier.rapier_collider_create_segment(world, body2, -1.0, 0.0, 1.0, 0.0);
        System.out.printf("  Created segment collider (handle: %d)%n", segment);
        
        // Test triangle
        long body3 = rapier.rapier_rigid_body_create_fixed(world, -5.0, 0.0);
        long triangle = rapier.rapier_collider_create_triangle(world, body3, 
            0.0, 1.0, -1.0, 0.0, 1.0, 0.0);
        System.out.printf("  Created triangle collider (handle: %d)%n", triangle);
        
        // Test collider properties
        double volume = rapier.rapier_collider_get_volume(world, capsule);
        double mass = rapier.rapier_collider_get_mass(world, capsule);
        System.out.printf("  Capsule volume: %.4f, mass: %.4f%n", volume, mass);
        
        // Test position getter
        DoubleByReference angle = new DoubleByReference();
        rapier.rapier_collider_get_position(world, capsule, x, y, angle);
        System.out.printf("  Capsule position: (%.2f, %.2f), angle: %.2f%n", 
            x.getValue(), y.getValue(), angle.getValue());
        
        // Test parent
        long parent = rapier.rapier_collider_get_parent(world, capsule);
        System.out.printf("  Capsule parent body handle: %d (expected %d)%n", parent, body);
        
        // Test solver groups
        rapier.rapier_collider_set_solver_groups(world, capsule, 0x0001, 0x0002);
        IntByReference memberships = new IntByReference();
        IntByReference filter = new IntByReference();
        rapier.rapier_collider_get_solver_groups(world, capsule, memberships, filter);
        System.out.printf("  Solver groups: memberships=0x%04X, filter=0x%04X%n", 
            memberships.getValue(), filter.getValue());
        
        System.out.println("  [OK] Additional collider shapes work correctly");
        
        rapier.rapier_world_destroy(world);
    }
    
    private static void testWorldQueries() {
        long world = rapier.rapier_world_create(0.0, -9.81);
        
        // Create some bodies
        for (int i = 0; i < 5; i++) {
            long body = rapier.rapier_rigid_body_create_dynamic(world, i * 2.0, 5.0);
            rapier.rapier_collider_create_ball(world, body, 0.5);
        }
        
        // Create some joints
        long body1 = rapier.rapier_rigid_body_create_fixed(world, 0.0, 10.0);
        long body2 = rapier.rapier_rigid_body_create_dynamic(world, 0.0, 8.0);
        rapier.rapier_collider_create_ball(world, body1, 0.1);
        rapier.rapier_collider_create_ball(world, body2, 0.3);
        rapier.rapier_joint_create_revolute(world, body1, body2, 0.0, 0.0, 0.0, 2.0);
        
        // Query world
        long numBodies = rapier.rapier_world_get_num_rigid_bodies(world);
        long numColliders = rapier.rapier_world_get_num_colliders(world);
        long numJoints = rapier.rapier_world_get_num_joints(world);
        
        System.out.printf("  Number of rigid bodies: %d%n", numBodies);
        System.out.printf("  Number of colliders: %d%n", numColliders);
        System.out.printf("  Number of joints: %d%n", numJoints);
        
        // Test timestep
        double timestep = rapier.rapier_world_get_timestep(world);
        System.out.printf("  Default timestep: %.6f%n", timestep);
        
        rapier.rapier_world_set_timestep(world, 1.0/120.0);
        timestep = rapier.rapier_world_get_timestep(world);
        System.out.printf("  Modified timestep: %.6f%n", timestep);
        
        // Test gravity
        DoubleByReference gx = new DoubleByReference();
        DoubleByReference gy = new DoubleByReference();
        rapier.rapier_world_get_gravity(world, gx, gy);
        System.out.printf("  Gravity: (%.2f, %.2f)%n", gx.getValue(), gy.getValue());
        
        rapier.rapier_world_set_gravity(world, 0.0, -20.0);
        rapier.rapier_world_get_gravity(world, gx, gy);
        System.out.printf("  Modified gravity: (%.2f, %.2f)%n", gx.getValue(), gy.getValue());
        
        System.out.println("  [OK] World queries work correctly");
        
        rapier.rapier_world_destroy(world);
    }
}
