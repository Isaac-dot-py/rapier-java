package com.rapier;

import com.sun.jna.Library;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.DoubleByReference;
import com.sun.jna.ptr.IntByReference;

/**
 * JNA interface to the Rapier native library.
 * This interface maps directly to the C FFI functions.
 * 
 * All functions operate on handles (long values) that represent native objects.
 * The user is responsible for managing handles and calling appropriate destroy functions.
 * 
 * Use {@link Rapier#create()} to obtain an instance of this interface.
 * 
 * == Enum Mappings ==
 * 
 * RigidBodyType: 0=Dynamic, 1=Fixed, 2=KinematicPositionBased, 3=KinematicVelocityBased
 * CoefficientCombineRule: 0=Average, 1=Min, 2=Multiply, 3=Max  
 * MotorModel: 0=AccelerationBased, 1=ForceBased
 * JointAxis: 0=X, 1=Y, 2=AngX
 */
public interface RapierNative extends Library {
    
    // ========================================================================
    // World Management
    // ========================================================================
    
    long rapier_world_create(double gravity_x, double gravity_y);
    void rapier_world_destroy(long world_handle);
    void rapier_world_step(long world_handle);
    boolean rapier_world_set_gravity(long world_handle, double gravity_x, double gravity_y);
    boolean rapier_world_get_gravity(long world_handle, DoubleByReference out_gravity_x, DoubleByReference out_gravity_y);
    boolean rapier_world_set_timestep(long world_handle, double timestep);
    double rapier_world_get_timestep(long world_handle);
    long rapier_world_get_num_rigid_bodies(long world_handle);
    long rapier_world_get_num_colliders(long world_handle);
    long rapier_world_get_num_joints(long world_handle);
    
    // ========================================================================
    // RigidBody Creation
    // ========================================================================
    
    long rapier_rigid_body_create_dynamic(long world_handle, double x, double y);
    long rapier_rigid_body_create_fixed(long world_handle, double x, double y);
    long rapier_rigid_body_create_kinematic_velocity_based(long world_handle, double x, double y);
    long rapier_rigid_body_create_kinematic_position_based(long world_handle, double x, double y);
    boolean rapier_rigid_body_remove(long world_handle, long body_handle);
    
    // ========================================================================
    // RigidBody Position & Rotation
    // ========================================================================
    
    boolean rapier_rigid_body_get_position(long world_handle, long body_handle, DoubleByReference out_x, DoubleByReference out_y);
    double rapier_rigid_body_get_rotation(long world_handle, long body_handle);
    boolean rapier_rigid_body_set_translation(long world_handle, long body_handle, double x, double y, boolean wake_up);
    boolean rapier_rigid_body_set_rotation(long world_handle, long body_handle, double angle, boolean wake_up);
    boolean rapier_rigid_body_set_next_kinematic_translation(long world_handle, long body_handle, double x, double y);
    boolean rapier_rigid_body_set_next_kinematic_rotation(long world_handle, long body_handle, double angle);
    
    // ========================================================================
    // RigidBody Velocity
    // ========================================================================
    
    boolean rapier_rigid_body_get_linvel(long world_handle, long body_handle, DoubleByReference out_vx, DoubleByReference out_vy);
    double rapier_rigid_body_get_angvel(long world_handle, long body_handle);
    boolean rapier_rigid_body_set_linvel(long world_handle, long body_handle, double vx, double vy, boolean wake_up);
    boolean rapier_rigid_body_set_angvel(long world_handle, long body_handle, double angvel, boolean wake_up);
    
    // ========================================================================
    // RigidBody Forces & Impulses
    // ========================================================================
    
    boolean rapier_rigid_body_apply_impulse(long world_handle, long body_handle, double impulse_x, double impulse_y, boolean wake_up);
    boolean rapier_rigid_body_apply_torque_impulse(long world_handle, long body_handle, double torque_impulse, boolean wake_up);
    boolean rapier_rigid_body_apply_impulse_at_point(long world_handle, long body_handle, double impulse_x, double impulse_y, double point_x, double point_y, boolean wake_up);
    boolean rapier_rigid_body_add_force(long world_handle, long body_handle, double fx, double fy, boolean wake_up);
    boolean rapier_rigid_body_add_torque(long world_handle, long body_handle, double torque, boolean wake_up);
    boolean rapier_rigid_body_add_force_at_point(long world_handle, long body_handle, double force_x, double force_y, double point_x, double point_y, boolean wake_up);
    boolean rapier_rigid_body_reset_forces(long world_handle, long body_handle, boolean wake_up);
    boolean rapier_rigid_body_reset_torques(long world_handle, long body_handle, boolean wake_up);
    
    // ========================================================================
    // RigidBody Mass & Inertia
    // ========================================================================
    
    boolean rapier_rigid_body_set_additional_mass(long world_handle, long body_handle, double additional_mass, boolean wake_up);
    double rapier_rigid_body_get_mass(long world_handle, long body_handle);
    double rapier_rigid_body_get_angular_inertia(long world_handle, long body_handle);
    
    // ========================================================================
    // RigidBody Damping
    // ========================================================================
    
    boolean rapier_rigid_body_set_linear_damping(long world_handle, long body_handle, double damping);
    double rapier_rigid_body_get_linear_damping(long world_handle, long body_handle);
    boolean rapier_rigid_body_set_angular_damping(long world_handle, long body_handle, double damping);
    double rapier_rigid_body_get_angular_damping(long world_handle, long body_handle);
    
    // ========================================================================
    // RigidBody Properties
    // ========================================================================
    
    /** @param body_type 0=Dynamic, 1=Fixed, 2=KinematicPositionBased, 3=KinematicVelocityBased */
    boolean rapier_rigid_body_set_body_type(long world_handle, long body_handle, int body_type, boolean wake_up);
    /** @return 0=Dynamic, 1=Fixed, 2=KinematicPositionBased, 3=KinematicVelocityBased */
    int rapier_rigid_body_get_body_type(long world_handle, long body_handle);
    boolean rapier_rigid_body_set_gravity_scale(long world_handle, long body_handle, double scale, boolean wake_up);
    double rapier_rigid_body_get_gravity_scale(long world_handle, long body_handle);
    boolean rapier_rigid_body_set_dominance_group(long world_handle, long body_handle, byte group);
    byte rapier_rigid_body_get_dominance_group(long world_handle, long body_handle);
    boolean rapier_rigid_body_set_enabled(long world_handle, long body_handle, boolean enabled);
    boolean rapier_rigid_body_is_enabled(long world_handle, long body_handle);
    
    // ========================================================================
    // RigidBody Sleep & CCD
    // ========================================================================
    
    boolean rapier_rigid_body_is_sleeping(long world_handle, long body_handle);
    boolean rapier_rigid_body_sleep(long world_handle, long body_handle);
    boolean rapier_rigid_body_wake_up(long world_handle, long body_handle, boolean strong);
    boolean rapier_rigid_body_is_ccd_enabled(long world_handle, long body_handle);
    boolean rapier_rigid_body_enable_ccd(long world_handle, long body_handle, boolean enabled);
    
    // ========================================================================
    // RigidBody Locked Axes
    // ========================================================================
    
    boolean rapier_rigid_body_lock_rotations(long world_handle, long body_handle, boolean locked, boolean wake_up);
    boolean rapier_rigid_body_lock_translations(long world_handle, long body_handle, boolean locked, boolean wake_up);
    boolean rapier_rigid_body_is_rotation_locked(long world_handle, long body_handle);
    boolean rapier_rigid_body_is_translation_locked(long world_handle, long body_handle);
    
    // ========================================================================
    // Collider Creation
    // ========================================================================
    
    long rapier_collider_create_cuboid(long world_handle, long body_handle, double half_width, double half_height);
    long rapier_collider_create_ball(long world_handle, long body_handle, double radius);
    long rapier_collider_create_capsule(long world_handle, long body_handle, double half_height, double radius);
    long rapier_collider_create_segment(long world_handle, long body_handle, double ax, double ay, double bx, double by);
    long rapier_collider_create_triangle(long world_handle, long body_handle, double ax, double ay, double bx, double by, double cx, double cy);
    long rapier_collider_create_heightfield(long world_handle, long body_handle, Pointer heights, long num_heights, double scale_x, double scale_y);
    boolean rapier_collider_remove(long world_handle, long collider_handle);
    
    // ========================================================================
    // Collider Material Properties
    // ========================================================================
    
    boolean rapier_collider_set_friction(long world_handle, long collider_handle, double friction);
    double rapier_collider_get_friction(long world_handle, long collider_handle);
    boolean rapier_collider_set_restitution(long world_handle, long collider_handle, double restitution);
    double rapier_collider_get_restitution(long world_handle, long collider_handle);
    
    /** @param rule 0=Average, 1=Min, 2=Multiply, 3=Max */
    boolean rapier_collider_set_friction_combine_rule(long world_handle, long collider_handle, int rule);
    /** @return 0=Average, 1=Min, 2=Multiply, 3=Max */
    int rapier_collider_get_friction_combine_rule(long world_handle, long collider_handle);
    /** @param rule 0=Average, 1=Min, 2=Multiply, 3=Max */
    boolean rapier_collider_set_restitution_combine_rule(long world_handle, long collider_handle, int rule);
    /** @return 0=Average, 1=Min, 2=Multiply, 3=Max */
    int rapier_collider_get_restitution_combine_rule(long world_handle, long collider_handle);
    
    // ========================================================================
    // Collider Sensor & Enabled
    // ========================================================================
    
    boolean rapier_collider_set_sensor(long world_handle, long collider_handle, boolean is_sensor);
    boolean rapier_collider_is_sensor(long world_handle, long collider_handle);
    boolean rapier_collider_set_enabled(long world_handle, long collider_handle, boolean enabled);
    boolean rapier_collider_is_enabled(long world_handle, long collider_handle);
    
    // ========================================================================
    // Collider Mass & Density
    // ========================================================================
    
    boolean rapier_collider_set_density(long world_handle, long collider_handle, double density);
    double rapier_collider_get_density(long world_handle, long collider_handle);
    boolean rapier_collider_set_mass(long world_handle, long collider_handle, double mass);
    double rapier_collider_get_mass(long world_handle, long collider_handle);
    double rapier_collider_get_volume(long world_handle, long collider_handle);
    
    // ========================================================================
    // Collider Position
    // ========================================================================
    
    boolean rapier_collider_get_position(long world_handle, long collider_handle, DoubleByReference out_x, DoubleByReference out_y, DoubleByReference out_angle);
    boolean rapier_collider_set_position(long world_handle, long collider_handle, double x, double y, double angle);
    boolean rapier_collider_set_translation(long world_handle, long collider_handle, double x, double y);
    boolean rapier_collider_set_rotation(long world_handle, long collider_handle, double angle);
    long rapier_collider_get_parent(long world_handle, long collider_handle);
    
    // ========================================================================
    // Collider Interaction Groups
    // ========================================================================
    
    boolean rapier_collider_set_collision_groups(long world_handle, long collider_handle, int memberships, int filter);
    boolean rapier_collider_get_collision_groups(long world_handle, long collider_handle, IntByReference out_memberships, IntByReference out_filter);
    boolean rapier_collider_set_solver_groups(long world_handle, long collider_handle, int memberships, int filter);
    boolean rapier_collider_get_solver_groups(long world_handle, long collider_handle, IntByReference out_memberships, IntByReference out_filter);
    
    // ========================================================================
    // Joint Creation
    // ========================================================================
    
    /** Creates a revolute (hinge) joint */
    long rapier_joint_create_revolute(long world_handle, long body_handle_1, long body_handle_2,
                                       double anchor1_x, double anchor1_y, double anchor2_x, double anchor2_y);
    
    /** Creates a prismatic (slider) joint */
    long rapier_joint_create_prismatic(long world_handle, long body_handle_1, long body_handle_2,
                                        double anchor1_x, double anchor1_y, double anchor2_x, double anchor2_y,
                                        double axis_x, double axis_y);
    
    /** Creates a fixed joint that locks relative motion */
    long rapier_joint_create_fixed(long world_handle, long body_handle_1, long body_handle_2,
                                    double anchor1_x, double anchor1_y, double anchor1_angle,
                                    double anchor2_x, double anchor2_y, double anchor2_angle);
    
    /** Creates a rope joint that limits maximum distance */
    long rapier_joint_create_rope(long world_handle, long body_handle_1, long body_handle_2,
                                   double anchor1_x, double anchor1_y, double anchor2_x, double anchor2_y,
                                   double max_distance);
    
    /** Creates a spring joint */
    long rapier_joint_create_spring(long world_handle, long body_handle_1, long body_handle_2,
                                     double anchor1_x, double anchor1_y, double anchor2_x, double anchor2_y,
                                     double rest_length, double stiffness, double damping);
    
    boolean rapier_joint_remove(long world_handle, long joint_handle);
    
    // ========================================================================
    // Joint Configuration
    // ========================================================================
    
    boolean rapier_joint_set_contacts_enabled(long world_handle, long joint_handle, boolean enabled);
    
    /** 
     * @param axis 0=X, 1=Y, 2=AngX
     * @param model 0=AccelerationBased, 1=ForceBased 
     */
    boolean rapier_joint_set_motor_model(long world_handle, long joint_handle, int axis, int model);
    
    /** @param axis 0=X, 1=Y, 2=AngX */
    boolean rapier_joint_set_motor_velocity(long world_handle, long joint_handle, int axis, double target_vel, double factor);
    
    /** @param axis 0=X, 1=Y, 2=AngX */
    boolean rapier_joint_set_motor_position(long world_handle, long joint_handle, int axis, double target_pos, double stiffness, double damping);
    
    /** @param axis 0=X, 1=Y, 2=AngX */
    boolean rapier_joint_set_motor_max_force(long world_handle, long joint_handle, int axis, double max_force);
    
    /** @param axis 0=X, 1=Y, 2=AngX */
    boolean rapier_joint_set_limits(long world_handle, long joint_handle, int axis, double min, double max);
}
