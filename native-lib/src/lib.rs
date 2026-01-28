use rapier2d_f64::prelude::*;
use rapier2d_f64::dynamics::CoefficientCombineRule;
use std::ops::{Deref, DerefMut};
use std::sync::{Mutex, OnceLock};

// Type aliases for handles
pub type WorldHandle = usize;
pub type RigidBodyHandle = usize;
pub type ColliderHandle = usize;
pub type JointHandle = usize;

// Global storage for physics world components
static WORLDS: OnceLock<Mutex<Vec<Option<PhysicsWorld>>>> = OnceLock::new();

struct PhysicsWorld {
    gravity: Vector<f64>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
}

fn get_worlds() -> &'static Mutex<Vec<Option<PhysicsWorld>>> {
    WORLDS.get_or_init(|| Mutex::new(Vec::new()))
}

struct WorldGuard<'a> {
    worlds: std::sync::MutexGuard<'a, Vec<Option<PhysicsWorld>>>,
    index: usize,
}

impl<'a> Deref for WorldGuard<'a> {
    type Target = PhysicsWorld;
    fn deref(&self) -> &Self::Target {
        self.worlds[self.index].as_ref().unwrap()
    }
}

impl<'a> DerefMut for WorldGuard<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.worlds[self.index].as_mut().unwrap()
    }
}

fn get_world(handle: WorldHandle) -> Option<WorldGuard<'static>> {
    let worlds = get_worlds().lock().ok()?;
    if handle < worlds.len() {
        if worlds[handle].is_some() {
            Some(WorldGuard { worlds, index: handle })
        } else {
            None
        }
    } else {
        None
    }
}

// World management functions
#[no_mangle]
pub extern "C" fn rapier_world_create(gravity_x: f64, gravity_y: f64) -> WorldHandle {
    let world = PhysicsWorld {
        gravity: vector![gravity_x, gravity_y],
        integration_parameters: IntegrationParameters::default(),
        physics_pipeline: PhysicsPipeline::new(),
        island_manager: IslandManager::new(),
        broad_phase: BroadPhase::new(),
        narrow_phase: NarrowPhase::new(),
        rigid_body_set: RigidBodySet::new(),
        collider_set: ColliderSet::new(),
        impulse_joint_set: ImpulseJointSet::new(),
        multibody_joint_set: MultibodyJointSet::new(),
        ccd_solver: CCDSolver::new(),
    };

    let mut worlds = get_worlds().lock().unwrap();
    let handle = worlds.len();
    worlds.push(Some(world));
    handle
}

#[no_mangle]
pub extern "C" fn rapier_world_destroy(handle: WorldHandle) {
    if let Ok(mut worlds) = get_worlds().lock() {
        if handle < worlds.len() {
            worlds[handle] = None;
        }
    }
}

#[no_mangle]
pub extern "C" fn rapier_world_step(handle: WorldHandle) {
    if let Some(mut world) = get_world(handle) {
        let PhysicsWorld {
            gravity,
            integration_parameters,
            physics_pipeline,
            island_manager,
            broad_phase,
            narrow_phase,
            rigid_body_set,
            collider_set,
            impulse_joint_set,
            multibody_joint_set,
            ccd_solver,
        } = &mut *world;

        physics_pipeline.step(
            gravity,
            integration_parameters,
            island_manager,
            broad_phase,
            narrow_phase,
            rigid_body_set,
            collider_set,
            impulse_joint_set,
            multibody_joint_set,
            ccd_solver,
            None,
            &(),
            &(),
        );
    }
}

// RigidBody functions
#[no_mangle]
pub extern "C" fn rapier_rigid_body_create_dynamic(
    world_handle: WorldHandle,
    x: f64,
    y: f64,
) -> RigidBodyHandle {
    if let Some(mut world) = get_world(world_handle) {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![x, y])
            .build();
        let handle = world.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_create_fixed(
    world_handle: WorldHandle,
    x: f64,
    y: f64,
) -> RigidBodyHandle {
    if let Some(mut world) = get_world(world_handle) {
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![x, y])
            .build();
        let handle = world.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_position(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    out_x: *mut f64,
    out_y: *mut f64,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            let pos = rb.translation();
            unsafe {
                *out_x = pos.x;
                *out_y = pos.y;
            }
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_rotation(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.rotation().angle();
        }
    }
    0.0
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_translation(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    x: f64,
    y: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_translation(vector![x, y], wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_linvel(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    vx: f64,
    vy: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_linvel(vector![vx, vy], wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_apply_impulse(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    impulse_x: f64,
    impulse_y: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.apply_impulse(vector![impulse_x, impulse_y], wake_up);
            return true;
        }
    }
    false
}

// Collider functions
#[no_mangle]
pub extern "C" fn rapier_collider_create_cuboid(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    half_width: f64,
    half_height: f64,
) -> ColliderHandle {
    if let Some(mut world) = get_world(world_handle) {
        let PhysicsWorld { collider_set, rigid_body_set, .. } = &mut *world;
        let rb_handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        let collider = ColliderBuilder::cuboid(half_width, half_height).build();
        let handle = collider_set.insert_with_parent(collider, rb_handle, rigid_body_set);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

#[no_mangle]
pub extern "C" fn rapier_collider_create_ball(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    radius: f64,
) -> ColliderHandle {
    if let Some(mut world) = get_world(world_handle) {
        let PhysicsWorld { collider_set, rigid_body_set, .. } = &mut *world;
        let rb_handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        let collider = ColliderBuilder::ball(radius).build();
        let handle = collider_set.insert_with_parent(collider, rb_handle, rigid_body_set);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

#[no_mangle]
pub extern "C" fn rapier_collider_set_restitution(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    restitution: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            collider.set_restitution(restitution);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_set_friction(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    friction: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            collider.set_friction(friction);
            return true;
        }
    }
    false
}

// Sensor functions
#[no_mangle]
pub extern "C" fn rapier_collider_set_sensor(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    is_sensor: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            collider.set_sensor(is_sensor);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_is_sensor(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            return collider.is_sensor();
        }
    }
    false
}

// Density functions
#[no_mangle]
pub extern "C" fn rapier_collider_set_density(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    density: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            collider.set_density(density);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_get_density(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            return collider.density();
        }
    }
    0.0
}

// Collision groups functions
#[no_mangle]
pub extern "C" fn rapier_collider_set_collision_groups(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    memberships: u32,
    filter: u32,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            let groups = InteractionGroups::new(memberships.into(), filter.into());
            collider.set_collision_groups(groups);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_get_collision_groups(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    out_memberships: *mut u32,
    out_filter: *mut u32,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            let groups = collider.collision_groups();
            unsafe {
                *out_memberships = groups.memberships.bits();
                *out_filter = groups.filter.bits();
            }
            return true;
        }
    }
    false
}

// Mass functions for rigid bodies
#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_additional_mass(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    additional_mass: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_additional_mass(additional_mass, wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_mass(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.mass();
        }
    }
    0.0
}

// Inertia functions for rigid bodies
#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_angular_inertia(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.mass_properties().local_mprops.principal_inertia();
        }
    }
    0.0
}

// Damping functions
#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_linear_damping(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    damping: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_linear_damping(damping);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_linear_damping(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.linear_damping();
        }
    }
    0.0
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_angular_damping(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    damping: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_angular_damping(damping);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_angular_damping(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.angular_damping();
        }
    }
    0.0
}

// ============================================================================
// Additional RigidBody Creation Functions
// ============================================================================

#[no_mangle]
pub extern "C" fn rapier_rigid_body_create_kinematic_velocity_based(
    world_handle: WorldHandle,
    x: f64,
    y: f64,
) -> RigidBodyHandle {
    if let Some(mut world) = get_world(world_handle) {
        let rigid_body = RigidBodyBuilder::kinematic_velocity_based()
            .translation(vector![x, y])
            .build();
        let handle = world.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_create_kinematic_position_based(
    world_handle: WorldHandle,
    x: f64,
    y: f64,
) -> RigidBodyHandle {
    if let Some(mut world) = get_world(world_handle) {
        let rigid_body = RigidBodyBuilder::kinematic_position_based()
            .translation(vector![x, y])
            .build();
        let handle = world.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_remove(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        let PhysicsWorld {
            island_manager,
            collider_set,
            impulse_joint_set,
            multibody_joint_set,
            rigid_body_set,
            ..
        } = &mut *world;

        rigid_body_set.remove(
            handle,
            island_manager,
            collider_set,
            impulse_joint_set,
            multibody_joint_set,
            true,
        );
        return true;
    }
    false
}

// ============================================================================
// RigidBody Velocity Functions
// ============================================================================

#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_linvel(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    out_vx: *mut f64,
    out_vy: *mut f64,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            let vel = rb.linvel();
            unsafe {
                *out_vx = vel.x;
                *out_vy = vel.y;
            }
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_angvel(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.angvel();
        }
    }
    0.0
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_angvel(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    angvel: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_angvel(angvel, wake_up);
            return true;
        }
    }
    false
}

// ============================================================================
// RigidBody Force/Torque Functions
// ============================================================================

#[no_mangle]
pub extern "C" fn rapier_rigid_body_apply_torque_impulse(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    torque_impulse: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.apply_torque_impulse(torque_impulse, wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_add_force(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    fx: f64,
    fy: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.add_force(vector![fx, fy], wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_add_torque(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    torque: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.add_torque(torque, wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_reset_forces(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.reset_forces(wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_reset_torques(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.reset_torques(wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_apply_impulse_at_point(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    impulse_x: f64,
    impulse_y: f64,
    point_x: f64,
    point_y: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.apply_impulse_at_point(vector![impulse_x, impulse_y], point![point_x, point_y], wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_add_force_at_point(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    force_x: f64,
    force_y: f64,
    point_x: f64,
    point_y: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.add_force_at_point(vector![force_x, force_y], point![point_x, point_y], wake_up);
            return true;
        }
    }
    false
}

// ============================================================================
// RigidBody Properties
// ============================================================================

#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_gravity_scale(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    scale: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_gravity_scale(scale, wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_gravity_scale(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.gravity_scale();
        }
    }
    1.0
}

/// Body types: 0 = Dynamic, 1 = Fixed, 2 = KinematicPositionBased, 3 = KinematicVelocityBased
#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_body_type(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    body_type: i32,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            let rb_type = match body_type {
                0 => RigidBodyType::Dynamic,
                1 => RigidBodyType::Fixed,
                2 => RigidBodyType::KinematicPositionBased,
                3 => RigidBodyType::KinematicVelocityBased,
                _ => return false,
            };
            rb.set_body_type(rb_type, wake_up);
            return true;
        }
    }
    false
}

/// Returns: 0 = Dynamic, 1 = Fixed, 2 = KinematicPositionBased, 3 = KinematicVelocityBased
#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_body_type(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> i32 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return match rb.body_type() {
                RigidBodyType::Dynamic => 0,
                RigidBodyType::Fixed => 1,
                RigidBodyType::KinematicPositionBased => 2,
                RigidBodyType::KinematicVelocityBased => 3,
            };
        }
    }
    -1
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_is_sleeping(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.is_sleeping();
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_sleep(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.sleep();
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_wake_up(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    strong: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.wake_up(strong);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_is_ccd_enabled(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.is_ccd_enabled();
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_enable_ccd(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    enabled: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.enable_ccd(enabled);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_dominance_group(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    group: i8,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_dominance_group(group);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_get_dominance_group(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> i8 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.dominance_group();
        }
    }
    0
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_lock_rotations(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    locked: bool,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.lock_rotations(locked, wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_lock_translations(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    locked: bool,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.lock_translations(locked, wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_is_rotation_locked(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.is_rotation_locked();
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_is_translation_locked(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.is_translation_locked();
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_enabled(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    enabled: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_enabled(enabled);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_is_enabled(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get(handle) {
            return rb.is_enabled();
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_rotation(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    angle: f64,
    wake_up: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_rotation(Rotation::new(angle), wake_up);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_next_kinematic_translation(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    x: f64,
    y: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_next_kinematic_translation(vector![x, y]);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_rigid_body_set_next_kinematic_rotation(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    angle: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        if let Some(rb) = world.rigid_body_set.get_mut(handle) {
            rb.set_next_kinematic_rotation(Rotation::new(angle));
            return true;
        }
    }
    false
}

// ============================================================================
// Additional Collider Creation Functions
// ============================================================================

#[no_mangle]
pub extern "C" fn rapier_collider_create_capsule(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    half_height: f64,
    radius: f64,
) -> ColliderHandle {
    if let Some(mut world) = get_world(world_handle) {
        let PhysicsWorld { collider_set, rigid_body_set, .. } = &mut *world;
        let rb_handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        let collider = ColliderBuilder::capsule_y(half_height, radius).build();
        let handle = collider_set.insert_with_parent(collider, rb_handle, rigid_body_set);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

#[no_mangle]
pub extern "C" fn rapier_collider_create_segment(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
) -> ColliderHandle {
    if let Some(mut world) = get_world(world_handle) {
        let PhysicsWorld { collider_set, rigid_body_set, .. } = &mut *world;
        let rb_handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        let collider = ColliderBuilder::segment(point![ax, ay], point![bx, by]).build();
        let handle = collider_set.insert_with_parent(collider, rb_handle, rigid_body_set);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

#[no_mangle]
pub extern "C" fn rapier_collider_create_triangle(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cx: f64,
    cy: f64,
) -> ColliderHandle {
    if let Some(mut world) = get_world(world_handle) {
        let PhysicsWorld { collider_set, rigid_body_set, .. } = &mut *world;
        let rb_handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        let collider = ColliderBuilder::triangle(point![ax, ay], point![bx, by], point![cx, cy]).build();
        let handle = collider_set.insert_with_parent(collider, rb_handle, rigid_body_set);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

#[no_mangle]
pub extern "C" fn rapier_collider_create_heightfield(
    world_handle: WorldHandle,
    body_handle: RigidBodyHandle,
    heights: *const f64,
    num_heights: usize,
    scale_x: f64,
    scale_y: f64,
) -> ColliderHandle {
    if heights.is_null() || num_heights == 0 {
        return usize::MAX;
    }
    if let Some(mut world) = get_world(world_handle) {
        let PhysicsWorld { collider_set, rigid_body_set, .. } = &mut *world;
        let rb_handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        let heights_slice = unsafe { std::slice::from_raw_parts(heights, num_heights) };
        let heights_vec = nalgebra::DVector::from_vec(heights_slice.to_vec());
        let collider = ColliderBuilder::heightfield(heights_vec, vector![scale_x, scale_y]).build();
        let handle = collider_set.insert_with_parent(collider, rb_handle, rigid_body_set);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

#[no_mangle]
pub extern "C" fn rapier_collider_remove(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        let PhysicsWorld {
            island_manager,
            rigid_body_set,
            collider_set,
            ..
        } = &mut *world;

        collider_set.remove(handle, island_manager, rigid_body_set, true);
        return true;
    }
    false
}

// ============================================================================
// Collider Material Properties
// ============================================================================

#[no_mangle]
pub extern "C" fn rapier_collider_get_friction(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            return collider.friction();
        }
    }
    0.0
}

#[no_mangle]
pub extern "C" fn rapier_collider_get_restitution(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            return collider.restitution();
        }
    }
    0.0
}

/// Combine rules: 0 = Average, 1 = Min, 2 = Multiply, 3 = Max
#[no_mangle]
pub extern "C" fn rapier_collider_set_friction_combine_rule(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    rule: i32,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            let combine_rule = match rule {
                0 => CoefficientCombineRule::Average,
                1 => CoefficientCombineRule::Min,
                2 => CoefficientCombineRule::Multiply,
                3 => CoefficientCombineRule::Max,
                _ => return false,
            };
            collider.set_friction_combine_rule(combine_rule);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_get_friction_combine_rule(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> i32 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            return match collider.friction_combine_rule() {
                CoefficientCombineRule::Average => 0,
                CoefficientCombineRule::Min => 1,
                CoefficientCombineRule::Multiply => 2,
                CoefficientCombineRule::Max => 3,
            };
        }
    }
    -1
}

#[no_mangle]
pub extern "C" fn rapier_collider_set_restitution_combine_rule(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    rule: i32,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            let combine_rule = match rule {
                0 => CoefficientCombineRule::Average,
                1 => CoefficientCombineRule::Min,
                2 => CoefficientCombineRule::Multiply,
                3 => CoefficientCombineRule::Max,
                _ => return false,
            };
            collider.set_restitution_combine_rule(combine_rule);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_get_restitution_combine_rule(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> i32 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            return match collider.restitution_combine_rule() {
                CoefficientCombineRule::Average => 0,
                CoefficientCombineRule::Min => 1,
                CoefficientCombineRule::Multiply => 2,
                CoefficientCombineRule::Max => 3,
            };
        }
    }
    -1
}

// ============================================================================
// Collider Solver Groups
// ============================================================================

#[no_mangle]
pub extern "C" fn rapier_collider_set_solver_groups(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    memberships: u32,
    filter: u32,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            let groups = InteractionGroups::new(memberships.into(), filter.into());
            collider.set_solver_groups(groups);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_get_solver_groups(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    out_memberships: *mut u32,
    out_filter: *mut u32,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            let groups = collider.solver_groups();
            unsafe {
                *out_memberships = groups.memberships.bits();
                *out_filter = groups.filter.bits();
            }
            return true;
        }
    }
    false
}

// ============================================================================
// Collider Position and Mass
// ============================================================================

#[no_mangle]
pub extern "C" fn rapier_collider_set_mass(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    mass: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            collider.set_mass(mass);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_get_mass(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            return collider.mass();
        }
    }
    0.0
}

#[no_mangle]
pub extern "C" fn rapier_collider_get_position(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    out_x: *mut f64,
    out_y: *mut f64,
    out_angle: *mut f64,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            let pos = collider.position();
            unsafe {
                *out_x = pos.translation.x;
                *out_y = pos.translation.y;
                *out_angle = pos.rotation.angle();
            }
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_set_position(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    x: f64,
    y: f64,
    angle: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            let isometry = Isometry::new(vector![x, y], angle);
            collider.set_position(isometry);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_set_translation(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    x: f64,
    y: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            collider.set_translation(vector![x, y]);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_set_rotation(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    angle: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            collider.set_rotation(Rotation::new(angle));
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_set_enabled(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
    enabled: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            collider.set_enabled(enabled);
            return true;
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_is_enabled(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            return collider.is_enabled();
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_collider_get_parent(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> RigidBodyHandle {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            if let Some(parent) = collider.parent() {
                return parent.into_raw_parts().0 as usize;
            }
        }
    }
    usize::MAX
}

#[no_mangle]
pub extern "C" fn rapier_collider_get_volume(
    world_handle: WorldHandle,
    collider_handle: ColliderHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get(handle) {
            return collider.volume();
        }
    }
    0.0
}

// ============================================================================
// Joint Creation Functions
// ============================================================================

/// Creates a revolute joint (hinge) between two bodies
#[no_mangle]
pub extern "C" fn rapier_joint_create_revolute(
    world_handle: WorldHandle,
    body_handle_1: RigidBodyHandle,
    body_handle_2: RigidBodyHandle,
    anchor1_x: f64,
    anchor1_y: f64,
    anchor2_x: f64,
    anchor2_y: f64,
) -> JointHandle {
    if let Some(mut world) = get_world(world_handle) {
        let rb1 = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle_1 as u32, 0);
        let rb2 = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle_2 as u32, 0);
        
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(point![anchor1_x, anchor1_y])
            .local_anchor2(point![anchor2_x, anchor2_y])
            .build();
        
        let handle = world.impulse_joint_set.insert(rb1, rb2, joint, true);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

/// Creates a prismatic joint (slider) between two bodies
#[no_mangle]
pub extern "C" fn rapier_joint_create_prismatic(
    world_handle: WorldHandle,
    body_handle_1: RigidBodyHandle,
    body_handle_2: RigidBodyHandle,
    anchor1_x: f64,
    anchor1_y: f64,
    anchor2_x: f64,
    anchor2_y: f64,
    axis_x: f64,
    axis_y: f64,
) -> JointHandle {
    if let Some(mut world) = get_world(world_handle) {
        let rb1 = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle_1 as u32, 0);
        let rb2 = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle_2 as u32, 0);
        
        let axis = UnitVector::new_normalize(vector![axis_x, axis_y]);
        let joint = PrismaticJointBuilder::new(axis)
            .local_anchor1(point![anchor1_x, anchor1_y])
            .local_anchor2(point![anchor2_x, anchor2_y])
            .build();
        
        let handle = world.impulse_joint_set.insert(rb1, rb2, joint, true);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

/// Creates a fixed joint between two bodies
#[no_mangle]
pub extern "C" fn rapier_joint_create_fixed(
    world_handle: WorldHandle,
    body_handle_1: RigidBodyHandle,
    body_handle_2: RigidBodyHandle,
    anchor1_x: f64,
    anchor1_y: f64,
    anchor1_angle: f64,
    anchor2_x: f64,
    anchor2_y: f64,
    anchor2_angle: f64,
) -> JointHandle {
    if let Some(mut world) = get_world(world_handle) {
        let rb1 = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle_1 as u32, 0);
        let rb2 = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle_2 as u32, 0);
        
        let joint = FixedJointBuilder::new()
            .local_frame1(Isometry::new(vector![anchor1_x, anchor1_y], anchor1_angle))
            .local_frame2(Isometry::new(vector![anchor2_x, anchor2_y], anchor2_angle))
            .build();
        
        let handle = world.impulse_joint_set.insert(rb1, rb2, joint, true);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

/// Creates a rope joint between two bodies
#[no_mangle]
pub extern "C" fn rapier_joint_create_rope(
    world_handle: WorldHandle,
    body_handle_1: RigidBodyHandle,
    body_handle_2: RigidBodyHandle,
    anchor1_x: f64,
    anchor1_y: f64,
    anchor2_x: f64,
    anchor2_y: f64,
    max_distance: f64,
) -> JointHandle {
    if let Some(mut world) = get_world(world_handle) {
        let rb1 = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle_1 as u32, 0);
        let rb2 = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle_2 as u32, 0);
        
        let joint = RopeJointBuilder::new(max_distance)
            .local_anchor1(point![anchor1_x, anchor1_y])
            .local_anchor2(point![anchor2_x, anchor2_y])
            .build();
        
        let handle = world.impulse_joint_set.insert(rb1, rb2, joint, true);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

/// Creates a spring joint between two bodies
#[no_mangle]
pub extern "C" fn rapier_joint_create_spring(
    world_handle: WorldHandle,
    body_handle_1: RigidBodyHandle,
    body_handle_2: RigidBodyHandle,
    anchor1_x: f64,
    anchor1_y: f64,
    anchor2_x: f64,
    anchor2_y: f64,
    rest_length: f64,
    stiffness: f64,
    damping: f64,
) -> JointHandle {
    if let Some(mut world) = get_world(world_handle) {
        let rb1 = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle_1 as u32, 0);
        let rb2 = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle_2 as u32, 0);
        
        let joint = SpringJointBuilder::new(rest_length, stiffness, damping)
            .local_anchor1(point![anchor1_x, anchor1_y])
            .local_anchor2(point![anchor2_x, anchor2_y])
            .build();
        
        let handle = world.impulse_joint_set.insert(rb1, rb2, joint, true);
        handle.into_raw_parts().0 as usize
    } else {
        usize::MAX
    }
}

/// Removes a joint from the world
#[no_mangle]
pub extern "C" fn rapier_joint_remove(
    world_handle: WorldHandle,
    joint_handle: JointHandle,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::ImpulseJointHandle::from_raw_parts(joint_handle as u32, 0);
        world.impulse_joint_set.remove(handle, true);
        return true;
    }
    false
}

// ============================================================================
// Joint Configuration Functions
// ============================================================================

#[no_mangle]
pub extern "C" fn rapier_joint_set_contacts_enabled(
    world_handle: WorldHandle,
    joint_handle: JointHandle,
    enabled: bool,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::ImpulseJointHandle::from_raw_parts(joint_handle as u32, 0);
        if let Some(joint) = world.impulse_joint_set.get_mut(handle) {
            joint.data.set_contacts_enabled(enabled);
            return true;
        }
    }
    false
}

/// Motor model: 0 = AccelerationBased, 1 = ForceBased
#[no_mangle]
pub extern "C" fn rapier_joint_set_motor_model(
    world_handle: WorldHandle,
    joint_handle: JointHandle,
    axis: i32,
    model: i32,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::ImpulseJointHandle::from_raw_parts(joint_handle as u32, 0);
        if let Some(joint) = world.impulse_joint_set.get_mut(handle) {
            let motor_model = match model {
                0 => MotorModel::AccelerationBased,
                1 => MotorModel::ForceBased,
                _ => return false,
            };
            let joint_axis = int_to_joint_axis(axis);
            if let Some(axis) = joint_axis {
                joint.data.set_motor_model(axis, motor_model);
                return true;
            }
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_joint_set_motor_velocity(
    world_handle: WorldHandle,
    joint_handle: JointHandle,
    axis: i32,
    target_vel: f64,
    factor: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::ImpulseJointHandle::from_raw_parts(joint_handle as u32, 0);
        if let Some(joint) = world.impulse_joint_set.get_mut(handle) {
            let joint_axis = int_to_joint_axis(axis);
            if let Some(axis) = joint_axis {
                joint.data.set_motor_velocity(axis, target_vel, factor);
                return true;
            }
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_joint_set_motor_position(
    world_handle: WorldHandle,
    joint_handle: JointHandle,
    axis: i32,
    target_pos: f64,
    stiffness: f64,
    damping: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::ImpulseJointHandle::from_raw_parts(joint_handle as u32, 0);
        if let Some(joint) = world.impulse_joint_set.get_mut(handle) {
            let joint_axis = int_to_joint_axis(axis);
            if let Some(axis) = joint_axis {
                joint.data.set_motor_position(axis, target_pos, stiffness, damping);
                return true;
            }
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_joint_set_motor_max_force(
    world_handle: WorldHandle,
    joint_handle: JointHandle,
    axis: i32,
    max_force: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::ImpulseJointHandle::from_raw_parts(joint_handle as u32, 0);
        if let Some(joint) = world.impulse_joint_set.get_mut(handle) {
            let joint_axis = int_to_joint_axis(axis);
            if let Some(axis) = joint_axis {
                joint.data.set_motor_max_force(axis, max_force);
                return true;
            }
        }
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_joint_set_limits(
    world_handle: WorldHandle,
    joint_handle: JointHandle,
    axis: i32,
    min: f64,
    max: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        let handle = rapier2d_f64::dynamics::ImpulseJointHandle::from_raw_parts(joint_handle as u32, 0);
        if let Some(joint) = world.impulse_joint_set.get_mut(handle) {
            let joint_axis = int_to_joint_axis(axis);
            if let Some(axis) = joint_axis {
                joint.data.set_limits(axis, [min, max]);
                return true;
            }
        }
    }
    false
}

/// Helper function to convert integer to JointAxis
/// 0 = X, 1 = Y, 2 = AngX
fn int_to_joint_axis(axis: i32) -> Option<JointAxis> {
    match axis {
        0 => Some(JointAxis::X),
        1 => Some(JointAxis::Y),
        2 => Some(JointAxis::AngX),
        _ => None,
    }
}

// ============================================================================
// World Configuration Functions
// ============================================================================

#[no_mangle]
pub extern "C" fn rapier_world_set_gravity(
    world_handle: WorldHandle,
    gravity_x: f64,
    gravity_y: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        world.gravity = vector![gravity_x, gravity_y];
        return true;
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_world_get_gravity(
    world_handle: WorldHandle,
    out_gravity_x: *mut f64,
    out_gravity_y: *mut f64,
) -> bool {
    if let Some(world) = get_world(world_handle) {
        unsafe {
            *out_gravity_x = world.gravity.x;
            *out_gravity_y = world.gravity.y;
        }
        return true;
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_world_set_timestep(
    world_handle: WorldHandle,
    timestep: f64,
) -> bool {
    if let Some(mut world) = get_world(world_handle) {
        world.integration_parameters.dt = timestep;
        return true;
    }
    false
}

#[no_mangle]
pub extern "C" fn rapier_world_get_timestep(
    world_handle: WorldHandle,
) -> f64 {
    if let Some(world) = get_world(world_handle) {
        return world.integration_parameters.dt;
    }
    0.0
}

#[no_mangle]
pub extern "C" fn rapier_world_get_num_rigid_bodies(
    world_handle: WorldHandle,
) -> usize {
    if let Some(world) = get_world(world_handle) {
        return world.rigid_body_set.len();
    }
    0
}

#[no_mangle]
pub extern "C" fn rapier_world_get_num_colliders(
    world_handle: WorldHandle,
) -> usize {
    if let Some(world) = get_world(world_handle) {
        return world.collider_set.len();
    }
    0
}

#[no_mangle]
pub extern "C" fn rapier_world_get_num_joints(
    world_handle: WorldHandle,
) -> usize {
    if let Some(world) = get_world(world_handle) {
        return world.impulse_joint_set.len();
    }
    0
}
