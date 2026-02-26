//! JNI bindings for rapier2d-f64 using handle-based ownership.
//!
//! All memory is owned by Rust; Java only stores opaque handles.

use std::cmp::max;
use std::sync::Mutex;

use jni::objects::{GlobalRef, JClass, JDoubleArray, JIntArray, JLongArray, JObject, JValue};
use jni::sys::{jboolean, jdouble, jint, jlong};
use jni::JNIEnv;
use jni::JavaVM;
use rapier2d_f64::geometry::{
    Aabb, BroadPhaseBvh, CollisionEvent, ContactForceEvent, Group, InteractionTestMode, Ray,
    SolverFlags,
};
use rapier2d_f64::pipeline::{
    ActiveEvents, ActiveHooks, EventHandler, PhysicsHooks, QueryFilter, QueryFilterFlags,
};
use rapier2d_f64::prelude::*;

const TRUE: jboolean = 1;
const FALSE: jboolean = 0;

struct WorldEvents {
    collision_events: Mutex<Vec<CollisionEvent>>,
    contact_force_events: Mutex<Vec<ContactForceEvent>>,
}

impl WorldEvents {
    fn new() -> Self {
        Self {
            collision_events: Mutex::new(Vec::new()),
            contact_force_events: Mutex::new(Vec::new()),
        }
    }
}

impl EventHandler for WorldEvents {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        if let Ok(mut events) = self.collision_events.lock() {
            events.push(event);
        }
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        let event = ContactForceEvent::from_contact_pair(dt, contact_pair, total_force_magnitude);
        if let Ok(mut events) = self.contact_force_events.lock() {
            events.push(event);
        }
    }
}

struct JavaHooks {
    java_vm: Option<JavaVM>,
    callback: Mutex<Option<GlobalRef>>,
}

impl JavaHooks {
    fn new(java_vm: Option<JavaVM>) -> Self {
        Self {
            java_vm,
            callback: Mutex::new(None),
        }
    }

    fn set_callback(&self, env: &JNIEnv, callback: JObject) {
        if let Ok(mut current) = self.callback.lock() {
            if callback.is_null() {
                *current = None;
            } else if let Ok(global) = env.new_global_ref(callback) {
                *current = Some(global);
            }
        }
    }

    fn with_callback<R, F>(&self, f: F) -> Option<R>
    where
        F: FnOnce(&mut JNIEnv, &JObject) -> Option<R>,
    {
        let vm = self.java_vm.as_ref()?;
        let callback = self.callback.lock().ok()?.clone()?;
        let mut env = vm.attach_current_thread().ok()?;
        f(&mut env, callback.as_obj())
    }
}

impl PhysicsHooks for JavaHooks {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        self.with_callback(|env, callback| {
            let collider1 = pack_handle_from_collider(context.collider1);
            let collider2 = pack_handle_from_collider(context.collider2);
            let body1 = pack_handle_from_rigid_body_option(context.rigid_body1);
            let body2 = pack_handle_from_rigid_body_option(context.rigid_body2);
            let result = env
                .call_method(
                    callback,
                    "filterContactPair",
                    "(JJJJ)I",
                    &[
                        JValue::Long(collider1),
                        JValue::Long(collider2),
                        JValue::Long(body1),
                        JValue::Long(body2),
                    ],
                )
                .ok()?
                .i()
                .ok()?;
            if result < 0 {
                None
            } else {
                Some(SolverFlags::from_bits_truncate(result as u32))
            }
        })
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        self.with_callback(|env, callback| {
            let collider1 = pack_handle_from_collider(context.collider1);
            let collider2 = pack_handle_from_collider(context.collider2);
            let body1 = pack_handle_from_rigid_body_option(context.rigid_body1);
            let body2 = pack_handle_from_rigid_body_option(context.rigid_body2);
            let result = env
                .call_method(
                    callback,
                    "filterIntersectionPair",
                    "(JJJJ)Z",
                    &[
                        JValue::Long(collider1),
                        JValue::Long(collider2),
                        JValue::Long(body1),
                        JValue::Long(body2),
                    ],
                )
                .ok()?
                .z()
                .ok()?;
            Some(result)
        })
        .unwrap_or(true)
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let _ = self.with_callback(|env, callback| {
            let collider1 = pack_handle_from_collider(context.collider1);
            let collider2 = pack_handle_from_collider(context.collider2);
            let body1 = pack_handle_from_rigid_body_option(context.rigid_body1);
            let body2 = pack_handle_from_rigid_body_option(context.rigid_body2);
            let normal_x = context.normal.x;
            let normal_y = context.normal.y;
            let result = env
                .call_method(
                    callback,
                    "modifySolverContactNormal",
                    "(JJJJDD)[D",
                    &[
                        JValue::Long(collider1),
                        JValue::Long(collider2),
                        JValue::Long(body1),
                        JValue::Long(body2),
                        JValue::Double(normal_x),
                        JValue::Double(normal_y),
                    ],
                )
                .ok()?;
            let array = result.l().ok()?;
            if array.is_null() {
                return Some(());
            }
            let array = JDoubleArray::from(array);
            let values = get_double_array(env, array)?;
            if values.len() >= 2 {
                context.normal.x = values[0];
                context.normal.y = values[1];
            }
            Some(())
        });
    }
}

#[repr(C)]
pub struct PhysicsWorld {
    gravity: Vector,
    pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    rigid_bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    integration_parameters: IntegrationParameters,
    events: WorldEvents,
    hooks: JavaHooks,
}

impl PhysicsWorld {
    fn new(java_vm: Option<JavaVM>) -> Self {
        Self {
            gravity: Vector::new(0.0, -9.81),
            pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            integration_parameters: IntegrationParameters::default(),
            events: WorldEvents::new(),
            hooks: JavaHooks::new(java_vm),
        }
    }
}

fn throw_illegal_state(env: &JNIEnv, message: &str) {
    if let Ok(mut env) = unsafe { JNIEnv::from_raw(env.get_native_interface()) } {
        let _ = env.throw_new("java/lang/IllegalStateException", message);
    }
}

fn bool_to_jboolean(value: bool) -> jboolean {
    if value {
        TRUE
    } else {
        FALSE
    }
}

fn pack_handle(index: u32, generation: u32) -> jlong {
    let packed = (generation as u64) << 32 | (index as u64 + 1);
    packed as i64
}

fn unpack_handle(handle: jlong) -> (u32, u32) {
    let raw = handle as u64;
    let index_raw = raw as u32;
    if index_raw == 0 {
        return (u32::MAX, u32::MAX);
    }
    let index = index_raw - 1;
    let generation = (raw >> 32) as u32;
    (index, generation)
}

fn pack_handle_from_rigid_body(handle: RigidBodyHandle) -> jlong {
    let (index, generation) = handle.into_raw_parts();
    pack_handle(index, generation)
}

fn pack_handle_from_rigid_body_option(handle: Option<RigidBodyHandle>) -> jlong {
    handle.map(pack_handle_from_rigid_body).unwrap_or(0)
}

fn pack_handle_from_collider(handle: ColliderHandle) -> jlong {
    let (index, generation) = handle.into_raw_parts();
    pack_handle(index, generation)
}

fn build_query_filter(flags: jint, memberships: jint, filter: jint) -> QueryFilter<'static> {
    let groups = if memberships != 0 || filter != 0 {
        Some(build_collision_groups(memberships, filter))
    } else {
        None
    };
    QueryFilter {
        flags: QueryFilterFlags::from_bits_truncate(flags as u32),
        groups,
        exclude_collider: None,
        exclude_rigid_body: None,
        predicate: None,
    }
}

fn rigid_body_handle(handle: jlong) -> RigidBodyHandle {
    if handle == 0 {
        return RigidBodyHandle::invalid();
    }
    let (index, generation) = unpack_handle(handle);
    RigidBodyHandle::from_raw_parts(index, generation)
}

fn collider_handle(handle: jlong) -> ColliderHandle {
    if handle == 0 {
        return ColliderHandle::invalid();
    }
    let (index, generation) = unpack_handle(handle);
    ColliderHandle::from_raw_parts(index, generation)
}

fn impulse_joint_handle(handle: jlong) -> ImpulseJointHandle {
    if handle == 0 {
        return ImpulseJointHandle::invalid();
    }
    let (index, generation) = unpack_handle(handle);
    ImpulseJointHandle::from_raw_parts(index, generation)
}

fn multibody_joint_handle(handle: jlong) -> MultibodyJointHandle {
    if handle == 0 {
        return MultibodyJointHandle::invalid();
    }
    let (index, generation) = unpack_handle(handle);
    MultibodyJointHandle::from_raw_parts(index, generation)
}

fn joint_axis_from_int(axis: jint) -> Option<JointAxis> {
    match axis {
        0 => Some(JointAxis::LinX),
        1 => Some(JointAxis::LinY),
        2 => Some(JointAxis::AngX),
        _ => None,
    }
}

fn impulse_joint_data_mut(
    world: &mut PhysicsWorld,
    handle: jlong,
) -> Option<&mut GenericJoint> {
    world
        .impulse_joints
        .get_mut(impulse_joint_handle(handle), true)
        .map(|joint| &mut joint.data)
}

fn impulse_joint_data(world: &PhysicsWorld, handle: jlong) -> Option<&GenericJoint> {
    world
        .impulse_joints
        .get(impulse_joint_handle(handle))
        .map(|joint| &joint.data)
}

fn multibody_joint_data_mut(
    world: &mut PhysicsWorld,
    handle: jlong,
) -> Option<&mut GenericJoint> {
    let (multibody, link_id) = world.multibody_joints.get_mut(multibody_joint_handle(handle))?;
    multibody
        .link_mut(link_id)
        .map(|link| &mut link.joint.data)
}

fn multibody_joint_data(world: &PhysicsWorld, handle: jlong) -> Option<&GenericJoint> {
    let (multibody, link_id) = world.multibody_joints.get(multibody_joint_handle(handle))?;
    multibody.link(link_id).map(|link| &link.joint.data)
}

unsafe fn world_from_handle<'a>(handle: jlong) -> Option<&'a mut PhysicsWorld> {
    if handle == 0 {
        return None;
    }
    Some(&mut *(handle as *mut PhysicsWorld))
}

fn get_double_array(env: &JNIEnv, array: JDoubleArray) -> Option<Vec<f64>> {
    let len = env.get_array_length(&array).ok()? as usize;
    let mut values = vec![0.0; len];
    if env.get_double_array_region(array, 0, &mut values).is_err() {
        return None;
    }
    Some(values)
}

fn get_int_array(env: &JNIEnv, array: JIntArray) -> Option<Vec<i32>> {
    let len = env.get_array_length(&array).ok()? as usize;
    let mut values = vec![0; len];
    if env.get_int_array_region(array, 0, &mut values).is_err() {
        return None;
    }
    Some(values)
}

fn build_collision_groups(memberships: jint, filter: jint) -> InteractionGroups {
    let memberships = Group::from_bits_truncate(memberships as u32);
    let filter = Group::from_bits_truncate(filter as u32);
    InteractionGroups::new(memberships, filter, InteractionTestMode::And)
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreate(
    env: JNIEnv,
    _class: JClass,
) -> jlong {
    let java_vm = env.get_java_vm().ok();
    let world = Box::new(PhysicsWorld::new(java_vm));
    Box::into_raw(world) as jlong
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldDestroy(
    _env: JNIEnv,
    _class: JClass,
    handle: jlong,
) {
    if handle == 0 {
        return;
    }
    unsafe {
        drop(Box::from_raw(handle as *mut PhysicsWorld));
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldSetGravity(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    x: jdouble,
    y: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        world.gravity = Vector::new(x, y);
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldStep(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    dt: jdouble,
    substeps: jint,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        world.integration_parameters.dt = dt;
        let substeps = max(1, substeps) as usize;
        for _ in 0..substeps {
            world.pipeline.step(
                world.gravity,
                &world.integration_parameters,
                &mut world.island_manager,
                &mut world.broad_phase,
                &mut world.narrow_phase,
                &mut world.rigid_bodies,
                &mut world.colliders,
                &mut world.impulse_joints,
                &mut world.multibody_joints,
                &mut world.ccd_solver,
                &world.hooks,
                &world.events,
            );
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldSetHooksCallback(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    callback: JObject,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        world.hooks.set_callback(&env, callback);
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreateRigidBody(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_type: jint,
    x: jdouble,
    y: jdouble,
    rotation: jdouble,
    vx: jdouble,
    vy: jdouble,
    ang_vel: jdouble,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let mut builder = match body_type {
            0 => RigidBodyBuilder::dynamic(),
            1 => RigidBodyBuilder::kinematic_velocity_based(),
            _ => RigidBodyBuilder::fixed(),
        };
        builder = builder
            .translation(Vector::new(x, y))
            .rotation(rotation)
            .linvel(Vector::new(vx, vy))
            .angvel(ang_vel);
        let handle = world.rigid_bodies.insert(builder.build());
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldRemoveRigidBody(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let rb_handle = rigid_body_handle(body_handle);
        world.rigid_bodies.remove(
            rb_handle,
            &mut world.island_manager,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
            true,
        );
    }
}

fn insert_collider(
    world: &mut PhysicsWorld,
    body_handle: jlong,
    mut builder: ColliderBuilder,
    sensor: bool,
    memberships: jint,
    filter: jint,
    density: jdouble,
    friction: jdouble,
    restitution: jdouble,
) -> ColliderHandle {
    builder = builder
        .sensor(sensor)
        .density(density)
        .friction(friction)
        .restitution(restitution)
        .collision_groups(build_collision_groups(memberships, filter));
    let collider = builder.build();
    if body_handle == 0 {
        world.colliders.insert(collider)
    } else {
        let parent = rigid_body_handle(body_handle);
        world
            .colliders
            .insert_with_parent(collider, parent, &mut world.rigid_bodies)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreateColliderCuboid(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    hx: jdouble,
    hy: jdouble,
    density: jdouble,
    friction: jdouble,
    restitution: jdouble,
    sensor: jboolean,
    memberships: jint,
    filter: jint,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let builder = ColliderBuilder::cuboid(hx, hy);
        let handle = insert_collider(
            world,
            body_handle,
            builder,
            sensor != 0,
            memberships,
            filter,
            density,
            friction,
            restitution,
        );
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreateColliderBall(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    radius: jdouble,
    density: jdouble,
    friction: jdouble,
    restitution: jdouble,
    sensor: jboolean,
    memberships: jint,
    filter: jint,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let builder = ColliderBuilder::ball(radius);
        let handle = insert_collider(
            world,
            body_handle,
            builder,
            sensor != 0,
            memberships,
            filter,
            density,
            friction,
            restitution,
        );
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreateColliderCapsule(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    half_height: jdouble,
    radius: jdouble,
    density: jdouble,
    friction: jdouble,
    restitution: jdouble,
    sensor: jboolean,
    memberships: jint,
    filter: jint,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let builder = ColliderBuilder::capsule_y(half_height, radius);
        let handle = insert_collider(
            world,
            body_handle,
            builder,
            sensor != 0,
            memberships,
            filter,
            density,
            friction,
            restitution,
        );
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreateColliderTriMesh(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    vertices: JDoubleArray,
    indices: JIntArray,
    density: jdouble,
    friction: jdouble,
    restitution: jdouble,
    sensor: jboolean,
    memberships: jint,
    filter: jint,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let vertices = match get_double_array(&env, vertices) {
            Some(values) => values,
            None => {
                throw_illegal_state(&env, "Failed to read vertices array.");
                return 0;
            }
        };
        if vertices.len() % 2 != 0 {
            throw_illegal_state(&env, "Vertices array must contain x/y pairs.");
            return 0;
        }
        let mut points = Vec::with_capacity(vertices.len() / 2);
        for chunk in vertices.chunks(2) {
            points.push(Vector::new(chunk[0], chunk[1]));
        }
        let indices = match get_int_array(&env, indices) {
            Some(values) => values,
            None => {
                throw_illegal_state(&env, "Failed to read indices array.");
                return 0;
            }
        };
        if indices.len() % 3 != 0 {
            throw_illegal_state(&env, "Indices array must be triangles.");
            return 0;
        }
        let mut triangles = Vec::with_capacity(indices.len() / 3);
        for chunk in indices.chunks(3) {
            triangles.push([chunk[0] as u32, chunk[1] as u32, chunk[2] as u32]);
        }
        let builder = match ColliderBuilder::trimesh(points, triangles) {
            Ok(builder) => builder,
            Err(_) => {
                throw_illegal_state(&env, "Failed to build trimesh collider.");
                return 0;
            }
        };
        let handle = insert_collider(
            world,
            body_handle,
            builder,
            sensor != 0,
            memberships,
            filter,
            density,
            friction,
            restitution,
        );
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreateColliderConvexHull(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    vertices: JDoubleArray,
    density: jdouble,
    friction: jdouble,
    restitution: jdouble,
    sensor: jboolean,
    memberships: jint,
    filter: jint,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let vertices = match get_double_array(&env, vertices) {
            Some(values) => values,
            None => {
                throw_illegal_state(&env, "Failed to read vertices array.");
                return 0;
            }
        };
        if vertices.len() % 2 != 0 {
            throw_illegal_state(&env, "Vertices array must contain x/y pairs.");
            return 0;
        }
        let mut points = Vec::with_capacity(vertices.len() / 2);
        for chunk in vertices.chunks(2) {
            points.push(Vector::new(chunk[0], chunk[1]));
        }
        let builder = match ColliderBuilder::convex_hull(&points) {
            Some(builder) => builder,
            None => {
                throw_illegal_state(&env, "Failed to build convex hull from points.");
                return 0;
            }
        };
        let handle = insert_collider(
            world,
            body_handle,
            builder,
            sensor != 0,
            memberships,
            filter,
            density,
            friction,
            restitution,
        );
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldRemoveCollider(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = collider_handle(collider);
        world
            .colliders
            .remove(handle, &mut world.island_manager, &mut world.rigid_bodies, true);
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreateBallJoint(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_a: jlong,
    body_b: jlong,
    ax: jdouble,
    ay: jdouble,
    bx: jdouble,
    by: jdouble,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(Vector::new(ax, ay))
            .local_anchor2(Vector::new(bx, by))
            .build();
        let handle = world.impulse_joints.insert(
            rigid_body_handle(body_a),
            rigid_body_handle(body_b),
            joint,
            true,
        );
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreateFixedJoint(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_a: jlong,
    body_b: jlong,
    ax: jdouble,
    ay: jdouble,
    arot: jdouble,
    bx: jdouble,
    by: jdouble,
    brot: jdouble,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let joint = FixedJointBuilder::new()
            .local_frame1(Pose::new(Vector::new(ax, ay), arot))
            .local_frame2(Pose::new(Vector::new(bx, by), brot))
            .build();
        let handle = world.impulse_joints.insert(
            rigid_body_handle(body_a),
            rigid_body_handle(body_b),
            joint,
            true,
        );
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreatePrismaticJoint(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_a: jlong,
    body_b: jlong,
    ax: jdouble,
    ay: jdouble,
    bx: jdouble,
    by: jdouble,
    axis_x: jdouble,
    axis_y: jdouble,
    limit_min: jdouble,
    limit_max: jdouble,
    stiffness: jdouble,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let axis = Vector::new(axis_x, axis_y);
        let joint = PrismaticJointBuilder::new(axis)
            .local_anchor1(Vector::new(ax, ay))
            .local_anchor2(Vector::new(bx, by))
            .limits([limit_min, limit_max])
            .build();
        let handle = world.impulse_joints.insert(
            rigid_body_handle(body_a),
            rigid_body_handle(body_b),
            joint,
            true,
        );
        if let Some(joint) = world.impulse_joints.get_mut(handle, true) {
            let axis = JointAxis::LinX;
            let index = axis as usize;
            joint.data.motor_axes |= axis.into();
            joint.data.motors[index].stiffness = stiffness;
        }
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreateRevoluteJoint(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_a: jlong,
    body_b: jlong,
    ax: jdouble,
    ay: jdouble,
    bx: jdouble,
    by: jdouble,
    limit_min: jdouble,
    limit_max: jdouble,
    stiffness: jdouble,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(Vector::new(ax, ay))
            .local_anchor2(Vector::new(bx, by))
            .limits([limit_min, limit_max])
            .build();
        let handle = world.impulse_joints.insert(
            rigid_body_handle(body_a),
            rigid_body_handle(body_b),
            joint,
            true,
        );
        if let Some(joint) = world.impulse_joints.get_mut(handle, true) {
            let axis = JointAxis::AngX;
            let index = axis as usize;
            joint.data.motor_axes |= axis.into();
            joint.data.motors[index].stiffness = stiffness;
        }
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldRemoveImpulseJoint(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = impulse_joint_handle(joint_handle);
        world.impulse_joints.remove(handle, true);
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCreateMultibodyRevoluteJoint(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_a: jlong,
    body_b: jlong,
    ax: jdouble,
    ay: jdouble,
    bx: jdouble,
    by: jdouble,
    limit_min: jdouble,
    limit_max: jdouble,
    stiffness: jdouble,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(Vector::new(ax, ay))
            .local_anchor2(Vector::new(bx, by))
            .limits([limit_min, limit_max])
            .build();
        let handle = world.multibody_joints.insert(
            rigid_body_handle(body_a),
            rigid_body_handle(body_b),
            joint,
            true,
        );
        let Some(handle) = handle else {
            return 0;
        };
        if let Some(joint) = world.multibody_joints.get_mut(handle).and_then(|(mb, id)| mb.link_mut(id)) {
            let axis = JointAxis::AngX;
            let index = axis as usize;
            joint.joint.data.motor_axes |= axis.into();
            joint.joint.data.motors[index].stiffness = stiffness;
        }
        let (index, generation) = handle.into_raw_parts();
        pack_handle(index, generation)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldRemoveMultibodyJoint(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = multibody_joint_handle(joint_handle);
        world.multibody_joints.remove(handle, true);
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_impulseJointGetLocalAnchor1X(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        impulse_joint_data(world, joint_handle)
            .map(|joint| joint.local_anchor1().x)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_impulseJointGetLocalAnchor1Y(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        impulse_joint_data(world, joint_handle)
            .map(|joint| joint.local_anchor1().y)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_impulseJointGetLocalAnchor2X(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        impulse_joint_data(world, joint_handle)
            .map(|joint| joint.local_anchor2().x)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_impulseJointGetLocalAnchor2Y(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        impulse_joint_data(world, joint_handle)
            .map(|joint| joint.local_anchor2().y)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_impulseJointSetLocalAnchors(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    ax: jdouble,
    ay: jdouble,
    bx: jdouble,
    by: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        if let Some(joint) = impulse_joint_data_mut(world, joint_handle) {
            joint.set_local_anchor1(Vector::new(ax, ay));
            joint.set_local_anchor2(Vector::new(bx, by));
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_impulseJointGetLimitMin(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    axis: jint,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let axis = match joint_axis_from_int(axis) {
            Some(axis) => axis,
            None => {
                throw_illegal_state(&env, "Invalid joint axis.");
                return 0.0;
            }
        };
        impulse_joint_data(world, joint_handle)
            .and_then(|joint| joint.limits(axis).map(|limits| limits.min))
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_impulseJointGetLimitMax(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    axis: jint,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let axis = match joint_axis_from_int(axis) {
            Some(axis) => axis,
            None => {
                throw_illegal_state(&env, "Invalid joint axis.");
                return 0.0;
            }
        };
        impulse_joint_data(world, joint_handle)
            .and_then(|joint| joint.limits(axis).map(|limits| limits.max))
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_impulseJointSetLimits(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    axis: jint,
    min: jdouble,
    max: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let axis = match joint_axis_from_int(axis) {
            Some(axis) => axis,
            None => {
                throw_illegal_state(&env, "Invalid joint axis.");
                return;
            }
        };
        if let Some(joint) = impulse_joint_data_mut(world, joint_handle) {
            joint.set_limits(axis, [min, max]);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_impulseJointGetMotorStiffness(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    axis: jint,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let axis = match joint_axis_from_int(axis) {
            Some(axis) => axis as usize,
            None => {
                throw_illegal_state(&env, "Invalid joint axis.");
                return 0.0;
            }
        };
        impulse_joint_data(world, joint_handle)
            .map(|joint| joint.motors[axis].stiffness)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_impulseJointSetMotorStiffness(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    axis: jint,
    stiffness: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let axis = match joint_axis_from_int(axis) {
            Some(axis) => axis,
            None => {
                throw_illegal_state(&env, "Invalid joint axis.");
                return;
            }
        };
        if let Some(joint) = impulse_joint_data_mut(world, joint_handle) {
            let index = axis as usize;
            joint.motor_axes |= axis.into();
            joint.motors[index].stiffness = stiffness;
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_multibodyJointGetLocalAnchor1X(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        multibody_joint_data(world, joint_handle)
            .map(|joint| joint.local_anchor1().x)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_multibodyJointGetLocalAnchor1Y(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        multibody_joint_data(world, joint_handle)
            .map(|joint| joint.local_anchor1().y)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_multibodyJointGetLocalAnchor2X(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        multibody_joint_data(world, joint_handle)
            .map(|joint| joint.local_anchor2().x)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_multibodyJointGetLocalAnchor2Y(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        multibody_joint_data(world, joint_handle)
            .map(|joint| joint.local_anchor2().y)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_multibodyJointSetLocalAnchors(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    ax: jdouble,
    ay: jdouble,
    bx: jdouble,
    by: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        if let Some(joint) = multibody_joint_data_mut(world, joint_handle) {
            joint.set_local_anchor1(Vector::new(ax, ay));
            joint.set_local_anchor2(Vector::new(bx, by));
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_multibodyJointGetLimitMin(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    axis: jint,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let axis = match joint_axis_from_int(axis) {
            Some(axis) => axis,
            None => {
                throw_illegal_state(&env, "Invalid joint axis.");
                return 0.0;
            }
        };
        multibody_joint_data(world, joint_handle)
            .and_then(|joint| joint.limits(axis).map(|limits| limits.min))
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_multibodyJointGetLimitMax(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    axis: jint,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let axis = match joint_axis_from_int(axis) {
            Some(axis) => axis,
            None => {
                throw_illegal_state(&env, "Invalid joint axis.");
                return 0.0;
            }
        };
        multibody_joint_data(world, joint_handle)
            .and_then(|joint| joint.limits(axis).map(|limits| limits.max))
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_multibodyJointSetLimits(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    axis: jint,
    min: jdouble,
    max: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let axis = match joint_axis_from_int(axis) {
            Some(axis) => axis,
            None => {
                throw_illegal_state(&env, "Invalid joint axis.");
                return;
            }
        };
        if let Some(joint) = multibody_joint_data_mut(world, joint_handle) {
            joint.set_limits(axis, [min, max]);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_multibodyJointGetMotorStiffness(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    axis: jint,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let axis = match joint_axis_from_int(axis) {
            Some(axis) => axis as usize,
            None => {
                throw_illegal_state(&env, "Invalid joint axis.");
                return 0.0;
            }
        };
        multibody_joint_data(world, joint_handle)
            .map(|joint| joint.motors[axis].stiffness)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_multibodyJointSetMotorStiffness(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    joint_handle: jlong,
    axis: jint,
    stiffness: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let axis = match joint_axis_from_int(axis) {
            Some(axis) => axis,
            None => {
                throw_illegal_state(&env, "Invalid joint axis.");
                return;
            }
        };
        if let Some(joint) = multibody_joint_data_mut(world, joint_handle) {
            let index = axis as usize;
            joint.motor_axes |= axis.into();
            joint.motors[index].stiffness = stiffness;
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyGetTranslationX(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = rigid_body_handle(body_handle);
        world
            .rigid_bodies
            .get(handle)
            .map(|rb| rb.translation().x)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyGetTranslationY(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = rigid_body_handle(body_handle);
        world
            .rigid_bodies
            .get(handle)
            .map(|rb| rb.translation().y)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyGetRotation(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = rigid_body_handle(body_handle);
        world
            .rigid_bodies
            .get(handle)
            .map(|rb| rb.rotation().angle())
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyGetLinvelX(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = rigid_body_handle(body_handle);
        world
            .rigid_bodies
            .get(handle)
            .map(|rb| rb.linvel().x)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyGetLinvelY(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = rigid_body_handle(body_handle);
        world
            .rigid_bodies
            .get(handle)
            .map(|rb| rb.linvel().y)
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyGetAngvel(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = rigid_body_handle(body_handle);
        world
            .rigid_bodies
            .get(handle)
            .map(|rb| rb.angvel())
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyGetMass(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = rigid_body_handle(body_handle);
        world
            .rigid_bodies
            .get(handle)
            .map(|rb| rb.mass())
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodySetTranslation(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    x: jdouble,
    y: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = rigid_body_handle(body_handle);
        if let Some(rb) = world.rigid_bodies.get_mut(handle) {
            rb.set_translation(Vector::new(x, y), true);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodySetRotation(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    angle: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = rigid_body_handle(body_handle);
        if let Some(rb) = world.rigid_bodies.get_mut(handle) {
            rb.set_rotation(Rotation::new(angle), true);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodySetLinvel(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    x: jdouble,
    y: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = rigid_body_handle(body_handle);
        if let Some(rb) = world.rigid_bodies.get_mut(handle) {
            rb.set_linvel(Vector::new(x, y), true);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodySetAngvel(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    angvel: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = rigid_body_handle(body_handle);
        if let Some(rb) = world.rigid_bodies.get_mut(handle) {
            rb.set_angvel(angvel, true);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodySetAdditionalMass(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    mass: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = rigid_body_handle(body_handle);
        if let Some(rb) = world.rigid_bodies.get_mut(handle) {
            rb.set_additional_mass(mass, true);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyGetLinearDamping(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = rigid_body_handle(body_handle);
        world
            .rigid_bodies
            .get(handle)
            .map(|rb| rb.linear_damping())
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodySetLinearDamping(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    damping: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = rigid_body_handle(body_handle);
        if let Some(rb) = world.rigid_bodies.get_mut(handle) {
            rb.set_linear_damping(damping);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyGetAngularDamping(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = rigid_body_handle(body_handle);
        world
            .rigid_bodies
            .get(handle)
            .map(|rb| rb.angular_damping())
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodySetAngularDamping(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    damping: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = rigid_body_handle(body_handle);
        if let Some(rb) = world.rigid_bodies.get_mut(handle) {
            rb.set_angular_damping(damping);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyAddForce(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    fx: jdouble,
    fy: jdouble,
    wake: jboolean,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = rigid_body_handle(body_handle);
        if let Some(rb) = world.rigid_bodies.get_mut(handle) {
            rb.add_force(Vector::new(fx, fy), wake != 0);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyAddForceAtPoint(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    fx: jdouble,
    fy: jdouble,
    px: jdouble,
    py: jdouble,
    wake: jboolean,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = rigid_body_handle(body_handle);
        if let Some(rb) = world.rigid_bodies.get_mut(handle) {
            rb.add_force_at_point(Vector::new(fx, fy), Vector::new(px, py), wake != 0);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_rigidBodyAddTorque(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    body_handle: jlong,
    torque: jdouble,
    wake: jboolean,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = rigid_body_handle(body_handle);
        if let Some(rb) = world.rigid_bodies.get_mut(handle) {
            rb.add_torque(torque, wake != 0);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderGetShapeType(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let handle = collider_handle(collider);
        let Some(collider) = world.colliders.get(handle) else {
            return 0;
        };
        match collider.shape().as_typed_shape() {
            TypedShape::Cuboid(_) => 1,
            TypedShape::Ball(_) => 2,
            TypedShape::Capsule(_) => 3,
            TypedShape::TriMesh(_) => 4,
            TypedShape::ConvexPolygon(_) => 5,
            _ => 0,
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderIsSensor(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) -> jboolean {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return FALSE;
            }
        };
        let handle = collider_handle(collider);
        world
            .colliders
            .get(handle)
            .map(|c| bool_to_jboolean(c.is_sensor()))
            .unwrap_or(FALSE)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderSetSensor(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
    sensor: jboolean,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = collider_handle(collider);
        if let Some(collider) = world.colliders.get_mut(handle) {
            collider.set_sensor(sensor != 0);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderGetDensity(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = collider_handle(collider);
        world
            .colliders
            .get(handle)
            .map(|c| c.density())
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderSetDensity(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
    density: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = collider_handle(collider);
        if let Some(collider) = world.colliders.get_mut(handle) {
            collider.set_density(density);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderGetFriction(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = collider_handle(collider);
        world
            .colliders
            .get(handle)
            .map(|c| c.friction())
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderSetFriction(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
    friction: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = collider_handle(collider);
        if let Some(collider) = world.colliders.get_mut(handle) {
            collider.set_friction(friction);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderGetRestitution(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) -> jdouble {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0.0;
            }
        };
        let handle = collider_handle(collider);
        world
            .colliders
            .get(handle)
            .map(|c| c.restitution())
            .unwrap_or(0.0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderSetRestitution(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
    restitution: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = collider_handle(collider);
        if let Some(collider) = world.colliders.get_mut(handle) {
            collider.set_restitution(restitution);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderGetRestitutionCombineRule(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let handle = collider_handle(collider);
        world
            .colliders
            .get(handle)
            .map(|c| c.restitution_combine_rule() as jint)
            .unwrap_or(0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderSetRestitutionCombineRule(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
    rule: jint,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = collider_handle(collider);
        if let Some(collider) = world.colliders.get_mut(handle) {
            let rule = match rule {
                1 => CoefficientCombineRule::Min,
                2 => CoefficientCombineRule::Multiply,
                3 => CoefficientCombineRule::Max,
                4 => CoefficientCombineRule::ClampedSum,
                _ => CoefficientCombineRule::Average,
            };
            collider.set_restitution_combine_rule(rule);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderGetActiveEvents(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let handle = collider_handle(collider);
        world
            .colliders
            .get(handle)
            .map(|c| c.active_events().bits() as jint)
            .unwrap_or(0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderSetActiveEvents(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
    events: jint,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = collider_handle(collider);
        if let Some(collider) = world.colliders.get_mut(handle) {
            collider.set_active_events(ActiveEvents::from_bits_truncate(events as u32));
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderGetActiveHooks(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let handle = collider_handle(collider);
        world
            .colliders
            .get(handle)
            .map(|c| c.active_hooks().bits() as jint)
            .unwrap_or(0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderSetActiveHooks(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
    hooks: jint,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = collider_handle(collider);
        if let Some(collider) = world.colliders.get_mut(handle) {
            collider.set_active_hooks(ActiveHooks::from_bits_truncate(hooks as u32));
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderSetContactForceEventThreshold(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
    threshold: jdouble,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = collider_handle(collider);
        if let Some(collider) = world.colliders.get_mut(handle) {
            collider.set_contact_force_event_threshold(threshold);
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderGetGroupMemberships(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let handle = collider_handle(collider);
        world
            .colliders
            .get(handle)
            .map(|c| c.collision_groups().memberships.bits() as jint)
            .unwrap_or(0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderGetGroupFilters(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let handle = collider_handle(collider);
        world
            .colliders
            .get(handle)
            .map(|c| c.collision_groups().filter.bits() as jint)
            .unwrap_or(0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_colliderSetCollisionGroups(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider: jlong,
    memberships: jint,
    filter: jint,
) {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return;
            }
        };
        let handle = collider_handle(collider);
        if let Some(collider) = world.colliders.get_mut(handle) {
            collider.set_collision_groups(build_collision_groups(memberships, filter));
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldAreColliding(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider_a: jlong,
    collider_b: jlong,
) -> jboolean {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return FALSE;
            }
        };
        let a = collider_handle(collider_a);
        let b = collider_handle(collider_b);
        let contact = world.narrow_phase.contact_pair(a, b);
        let intersection = world.narrow_phase.intersection_pair(a, b).unwrap_or(false);
        bool_to_jboolean(contact.is_some() || intersection)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldGetCollisionEventCount(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        world
            .events
            .collision_events
            .lock()
            .map(|events| events.len() as jint)
            .unwrap_or(0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldDrainCollisionEvents(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider_pairs: JLongArray,
    flags_out: JIntArray,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let Ok(mut events) = world.events.collision_events.lock() else {
            return 0;
        };
        let pair_capacity = env.get_array_length(&collider_pairs).unwrap_or(0) / 2;
        let flags_capacity = env.get_array_length(&flags_out).unwrap_or(0);
        let count = events
            .len()
            .min(pair_capacity as usize)
            .min(flags_capacity as usize);
        let drained: Vec<CollisionEvent> = events.drain(..count).collect();
        let mut pair_values = Vec::with_capacity(count * 2);
        let mut flag_values = Vec::with_capacity(count);
        for event in drained {
            let (c1, c2, flags, started) = match event {
                CollisionEvent::Started(c1, c2, flags) => (c1, c2, flags, true),
                CollisionEvent::Stopped(c1, c2, flags) => (c1, c2, flags, false),
            };
            pair_values.push(pack_handle_from_collider(c1));
            pair_values.push(pack_handle_from_collider(c2));
            let mut encoded = (flags.bits() as jint) << 1;
            if started {
                encoded |= 1;
            }
            flag_values.push(encoded);
        }
        if env
            .set_long_array_region(collider_pairs, 0, &pair_values)
            .is_err()
            || env
                .set_int_array_region(flags_out, 0, &flag_values)
                .is_err()
        {
            throw_illegal_state(&env, "Failed to write collision events.");
            return 0;
        }
        count as jint
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldGetContactForceEventCount(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        world
            .events
            .contact_force_events
            .lock()
            .map(|events| events.len() as jint)
            .unwrap_or(0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldDrainContactForceEvents(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    collider_pairs: JLongArray,
    data_out: JDoubleArray,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let Ok(mut events) = world.events.contact_force_events.lock() else {
            return 0;
        };
        let pair_capacity = env.get_array_length(&collider_pairs).unwrap_or(0) / 2;
        let data_capacity = env.get_array_length(&data_out).unwrap_or(0) / 6;
        let count = events
            .len()
            .min(pair_capacity as usize)
            .min(data_capacity as usize);
        let drained: Vec<ContactForceEvent> = events.drain(..count).collect();
        let mut pair_values = Vec::with_capacity(count * 2);
        let mut data_values = Vec::with_capacity(count * 6);
        for event in drained {
            pair_values.push(pack_handle_from_collider(event.collider1));
            pair_values.push(pack_handle_from_collider(event.collider2));
            data_values.push(event.total_force.x);
            data_values.push(event.total_force.y);
            data_values.push(event.total_force_magnitude);
            data_values.push(event.max_force_direction.x);
            data_values.push(event.max_force_direction.y);
            data_values.push(event.max_force_magnitude);
        }
        if env
            .set_long_array_region(collider_pairs, 0, &pair_values)
            .is_err()
            || env
                .set_double_array_region(data_out, 0, &data_values)
                .is_err()
        {
            throw_illegal_state(&env, "Failed to write contact force events.");
            return 0;
        }
        count as jint
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCastRay(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    ox: jdouble,
    oy: jdouble,
    dx: jdouble,
    dy: jdouble,
    max_toi: jdouble,
    solid: jboolean,
    memberships: jint,
    filter: jint,
    query_flags: jint,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let query_filter = build_query_filter(query_flags, memberships, filter);
        let query_pipeline = world.broad_phase.as_query_pipeline(
            world.narrow_phase.query_dispatcher(),
            &world.rigid_bodies,
            &world.colliders,
            query_filter,
        );
        let ray = Ray::new(Vector::new(ox, oy), Vector::new(dx, dy));
        query_pipeline
            .cast_ray(&ray, max_toi, solid != 0)
            .map(|(handle, _)| pack_handle_from_collider(handle))
            .unwrap_or(0)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldCastRayAndGetNormal(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    ox: jdouble,
    oy: jdouble,
    dx: jdouble,
    dy: jdouble,
    max_toi: jdouble,
    solid: jboolean,
    memberships: jint,
    filter: jint,
    query_flags: jint,
    out: JDoubleArray,
) -> jlong {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let query_filter = build_query_filter(query_flags, memberships, filter);
        let query_pipeline = world.broad_phase.as_query_pipeline(
            world.narrow_phase.query_dispatcher(),
            &world.rigid_bodies,
            &world.colliders,
            query_filter,
        );
        let ray = Ray::new(Vector::new(ox, oy), Vector::new(dx, dy));
        let Some((handle, intersection)) =
            query_pipeline.cast_ray_and_get_normal(&ray, max_toi, solid != 0)
        else {
            return 0;
        };
        if env.get_array_length(&out).unwrap_or(0) >= 3 {
            let values = [
                intersection.time_of_impact,
                intersection.normal.x,
                intersection.normal.y,
            ];
            if env.set_double_array_region(out, 0, &values).is_err() {
                throw_illegal_state(&env, "Failed to write ray cast output.");
            }
        }
        pack_handle_from_collider(handle)
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldQueryAabb(
    env: JNIEnv,
    _class: JClass,
    world_handle: jlong,
    min_x: jdouble,
    min_y: jdouble,
    max_x: jdouble,
    max_y: jdouble,
    memberships: jint,
    filter: jint,
    query_flags: jint,
    out: JLongArray,
) -> jint {
    unsafe {
        let world = match world_from_handle(world_handle) {
            Some(world) => world,
            None => {
                throw_illegal_state(&env, "World handle is null.");
                return 0;
            }
        };
        let query_filter = build_query_filter(query_flags, memberships, filter);
        let query_pipeline = world.broad_phase.as_query_pipeline(
            world.narrow_phase.query_dispatcher(),
            &world.rigid_bodies,
            &world.colliders,
            query_filter,
        );
    let aabb = Aabb::new(Vector::new(min_x, min_y), Vector::new(max_x, max_y));
        let mut handles = Vec::new();
        for (handle, _) in query_pipeline.intersect_aabb_conservative(aabb) {
            handles.push(pack_handle_from_collider(handle));
        }
        let capacity = env.get_array_length(&out).unwrap_or(0) as usize;
        let count = handles.len().min(capacity);
        if env
            .set_long_array_region(out, 0, &handles[..count])
            .is_err()
        {
            throw_illegal_state(&env, "Failed to write AABB query output.");
            return 0;
        }
        count as jint
    }
}

#[no_mangle]
pub extern "system" fn Java_com_rapier_NativeBindings_worldComputeFreeFallY(
    _env: JNIEnv,
    _class: JClass,
    initial_y: jdouble,
    gravity_y: jdouble,
    time: jdouble,
) -> jdouble {
    initial_y + 0.5 * gravity_y * time * time
}
