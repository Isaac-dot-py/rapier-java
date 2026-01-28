use rapier2d_f64::prelude::*;

// Type aliases for handles
pub type WorldHandle = usize;
pub type RigidBodyHandle = usize;
pub type ColliderHandle = usize;

// Global storage for physics world components
static mut WORLDS: Option<Vec<Option<PhysicsWorld>>> = None;

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

fn get_worlds() -> &'static mut Vec<Option<PhysicsWorld>> {
    unsafe {
        if WORLDS.is_none() {
            WORLDS = Some(Vec::new());
        }
        WORLDS.as_mut().unwrap()
    }
}

fn get_world(handle: WorldHandle) -> Option<&'static mut PhysicsWorld> {
    let worlds = get_worlds();
    if handle < worlds.len() {
        worlds[handle].as_mut()
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

    let worlds = get_worlds();
    let handle = worlds.len();
    worlds.push(Some(world));
    handle
}

#[no_mangle]
pub extern "C" fn rapier_world_destroy(handle: WorldHandle) {
    let worlds = get_worlds();
    if handle < worlds.len() {
        worlds[handle] = None;
    }
}

#[no_mangle]
pub extern "C" fn rapier_world_step(handle: WorldHandle) {
    if let Some(world) = get_world(handle) {
        world.physics_pipeline.step(
            &world.gravity,
            &world.integration_parameters,
            &mut world.island_manager,
            &mut world.broad_phase,
            &mut world.narrow_phase,
            &mut world.rigid_body_set,
            &mut world.collider_set,
            &mut world.impulse_joint_set,
            &mut world.multibody_joint_set,
            &mut world.ccd_solver,
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
    if let Some(world) = get_world(world_handle) {
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
    if let Some(world) = get_world(world_handle) {
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
    if let Some(world) = get_world(world_handle) {
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
    if let Some(world) = get_world(world_handle) {
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
    if let Some(world) = get_world(world_handle) {
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
    if let Some(world) = get_world(world_handle) {
        let rb_handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        let collider = ColliderBuilder::cuboid(half_width, half_height).build();
        let handle = world.collider_set.insert_with_parent(collider, rb_handle, &mut world.rigid_body_set);
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
    if let Some(world) = get_world(world_handle) {
        let rb_handle = rapier2d_f64::dynamics::RigidBodyHandle::from_raw_parts(body_handle as u32, 0);
        let collider = ColliderBuilder::ball(radius).build();
        let handle = world.collider_set.insert_with_parent(collider, rb_handle, &mut world.rigid_body_set);
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
    if let Some(world) = get_world(world_handle) {
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
    if let Some(world) = get_world(world_handle) {
        let handle = rapier2d_f64::geometry::ColliderHandle::from_raw_parts(collider_handle as u32, 0);
        if let Some(collider) = world.collider_set.get_mut(handle) {
            collider.set_friction(friction);
            return true;
        }
    }
    false
}
