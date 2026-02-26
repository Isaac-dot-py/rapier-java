package com.rapier;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

public final class NativeBindings {

    static {
        loadNative();
    }

    private static void loadNative() {
        String os = normalizeOS();
        String arch = normalizeArch();

        String libName = System.mapLibraryName("rapier2d_jni");
        String resourcePath = "/natives/" + os + "-" + arch + "/" + libName;

        try (InputStream in = NativeBindings.class.getResourceAsStream(resourcePath)) {
            if (in == null) {
                throw new UnsatisfiedLinkError("Native library not found: " + resourcePath);
            }

            File temp = File.createTempFile("rapier-", libName);
            temp.deleteOnExit();

            Files.copy(in, temp.toPath(), StandardCopyOption.REPLACE_EXISTING);
            System.load(temp.getAbsolutePath());

        } catch (IOException e) {
            throw new RuntimeException("Failed to load native library", e);
        }
    }

    private static String normalizeOS() {
        String os = System.getProperty("os.name").toLowerCase();
        if (os.contains("win")) return "windows";
        if (os.contains("mac")) return "macos";
        return "linux";
    }

    private static String normalizeArch() {
        String arch = System.getProperty("os.arch").toLowerCase();
        if (arch.equals("amd64")) return "x86_64";
        if (arch.equals("aarch64")) return "aarch64";
        return arch;
    }

    private NativeBindings() {}

    public static final int RIGID_BODY_DYNAMIC = 0;
    public static final int RIGID_BODY_KINEMATIC_VELOCITY = 1;
    public static final int RIGID_BODY_FIXED = 2;

    public static final int JOINT_AXIS_LIN_X = 0;
    public static final int JOINT_AXIS_LIN_Y = 1;
    public static final int JOINT_AXIS_ANG_X = 2;

    public static final int COLLIDER_SHAPE_CUBOID = 1;
    public static final int COLLIDER_SHAPE_BALL = 2;
    public static final int COLLIDER_SHAPE_CAPSULE = 3;
    public static final int COLLIDER_SHAPE_TRIMESH = 4;
    public static final int COLLIDER_SHAPE_CONVEX = 5;

    public static final int COEFF_COMBINE_AVERAGE = 0;
    public static final int COEFF_COMBINE_MIN = 1;
    public static final int COEFF_COMBINE_MULTIPLY = 2;
    public static final int COEFF_COMBINE_MAX = 3;
    public static final int COEFF_COMBINE_CLAMPED_SUM = 4;

    public static final int ACTIVE_EVENTS_COLLISION = 0b0001;
    public static final int ACTIVE_EVENTS_CONTACT_FORCE = 0b0010;

    public static final int ACTIVE_HOOK_FILTER_CONTACT_PAIRS = 0b0001;
    public static final int ACTIVE_HOOK_FILTER_INTERSECTION_PAIR = 0b0010;
    public static final int ACTIVE_HOOK_MODIFY_SOLVER_CONTACTS = 0b0100;

    public static final int SOLVER_FLAG_COMPUTE_IMPULSES = 0b001;

    public static final int COLLISION_EVENT_FLAG_SENSOR = 0b0001;
    public static final int COLLISION_EVENT_FLAG_REMOVED = 0b0010;

    public static final int QUERY_FILTER_EXCLUDE_FIXED = 0b0001;
    public static final int QUERY_FILTER_EXCLUDE_KINEMATIC = 0b0010;
    public static final int QUERY_FILTER_EXCLUDE_DYNAMIC = 0b0100;
    public static final int QUERY_FILTER_EXCLUDE_SENSORS = 0b1000;
    public static final int QUERY_FILTER_EXCLUDE_SOLIDS = 0b1_0000;

    public interface PhysicsHooksCallback {
        int filterContactPair(long collider1, long collider2, long body1, long body2);

        boolean filterIntersectionPair(long collider1, long collider2, long body1, long body2);

        double[] modifySolverContactNormal(
                long collider1, long collider2, long body1, long body2, double normalX, double normalY);
    }

    public static native long worldCreate();

    public static native void worldDestroy(long worldHandle);

    public static native void worldSetGravity(long worldHandle, double x, double y);

    public static native void worldStep(long worldHandle, double dt, int substeps);

    public static native void worldSetHooksCallback(long worldHandle, PhysicsHooksCallback callback);

    public static native long worldCreateRigidBody(
            long worldHandle, int bodyType, double x, double y, double rotation, double vx, double vy, double angVel);

    public static native void worldRemoveRigidBody(long worldHandle, long bodyHandle);

    public static native long worldCreateColliderCuboid(
            long worldHandle,
            long bodyHandle,
            double hx,
            double hy,
            double density,
            double friction,
            double restitution,
            boolean sensor,
            int memberships,
            int filter);

    public static native long worldCreateColliderBall(
            long worldHandle,
            long bodyHandle,
            double radius,
            double density,
            double friction,
            double restitution,
            boolean sensor,
            int memberships,
            int filter);

    public static native long worldCreateColliderCapsule(
            long worldHandle,
            long bodyHandle,
            double halfHeight,
            double radius,
            double density,
            double friction,
            double restitution,
            boolean sensor,
            int memberships,
            int filter);

    public static native long worldCreateColliderTriMesh(
            long worldHandle,
            long bodyHandle,
            double[] vertices,
            int[] indices,
            double density,
            double friction,
            double restitution,
            boolean sensor,
            int memberships,
            int filter);

    public static native long worldCreateColliderConvexHull(
            long worldHandle,
            long bodyHandle,
            double[] vertices,
            double density,
            double friction,
            double restitution,
            boolean sensor,
            int memberships,
            int filter);

    public static native void worldRemoveCollider(long worldHandle, long colliderHandle);

    public static native long worldCreateBallJoint(
            long worldHandle, long bodyA, long bodyB, double ax, double ay, double bx, double by);

    public static native long worldCreateFixedJoint(
            long worldHandle,
            long bodyA,
            long bodyB,
            double ax,
            double ay,
            double aRot,
            double bx,
            double by,
            double bRot);

    public static native long worldCreatePrismaticJoint(
            long worldHandle,
            long bodyA,
            long bodyB,
            double ax,
            double ay,
            double bx,
            double by,
            double axisX,
            double axisY,
            double limitMin,
            double limitMax,
            double stiffness);

    public static native long worldCreateRevoluteJoint(
            long worldHandle,
            long bodyA,
            long bodyB,
            double ax,
            double ay,
            double bx,
            double by,
            double limitMin,
            double limitMax,
            double stiffness);

    public static native void worldRemoveImpulseJoint(long worldHandle, long jointHandle);

    public static native long worldCreateMultibodyRevoluteJoint(
            long worldHandle,
            long bodyA,
            long bodyB,
            double ax,
            double ay,
            double bx,
            double by,
            double limitMin,
            double limitMax,
            double stiffness);

    public static native void worldRemoveMultibodyJoint(long worldHandle, long jointHandle);

    public static native double impulseJointGetLocalAnchor1X(long worldHandle, long jointHandle);

    public static native double impulseJointGetLocalAnchor1Y(long worldHandle, long jointHandle);

    public static native double impulseJointGetLocalAnchor2X(long worldHandle, long jointHandle);

    public static native double impulseJointGetLocalAnchor2Y(long worldHandle, long jointHandle);

    public static native void impulseJointSetLocalAnchors(
            long worldHandle, long jointHandle, double ax, double ay, double bx, double by);

    public static native double impulseJointGetLimitMin(long worldHandle, long jointHandle, int axis);

    public static native double impulseJointGetLimitMax(long worldHandle, long jointHandle, int axis);

    public static native void impulseJointSetLimits(
            long worldHandle, long jointHandle, int axis, double min, double max);

    public static native double impulseJointGetMotorStiffness(long worldHandle, long jointHandle, int axis);

    public static native void impulseJointSetMotorStiffness(
            long worldHandle, long jointHandle, int axis, double stiffness);

    public static native double multibodyJointGetLocalAnchor1X(long worldHandle, long jointHandle);

    public static native double multibodyJointGetLocalAnchor1Y(long worldHandle, long jointHandle);

    public static native double multibodyJointGetLocalAnchor2X(long worldHandle, long jointHandle);

    public static native double multibodyJointGetLocalAnchor2Y(long worldHandle, long jointHandle);

    public static native void multibodyJointSetLocalAnchors(
            long worldHandle, long jointHandle, double ax, double ay, double bx, double by);

    public static native double multibodyJointGetLimitMin(long worldHandle, long jointHandle, int axis);

    public static native double multibodyJointGetLimitMax(long worldHandle, long jointHandle, int axis);

    public static native void multibodyJointSetLimits(
            long worldHandle, long jointHandle, int axis, double min, double max);

    public static native double multibodyJointGetMotorStiffness(long worldHandle, long jointHandle, int axis);

    public static native void multibodyJointSetMotorStiffness(
            long worldHandle, long jointHandle, int axis, double stiffness);

    public static native double rigidBodyGetTranslationX(long worldHandle, long bodyHandle);

    public static native double rigidBodyGetTranslationY(long worldHandle, long bodyHandle);

    public static native double rigidBodyGetRotation(long worldHandle, long bodyHandle);

    public static native double rigidBodyGetLinvelX(long worldHandle, long bodyHandle);

    public static native double rigidBodyGetLinvelY(long worldHandle, long bodyHandle);

    public static native double rigidBodyGetAngvel(long worldHandle, long bodyHandle);

    public static native double rigidBodyGetMass(long worldHandle, long bodyHandle);

    public static native void rigidBodySetTranslation(long worldHandle, long bodyHandle, double x, double y);

    public static native void rigidBodySetRotation(long worldHandle, long bodyHandle, double angle);

    public static native void rigidBodySetLinvel(long worldHandle, long bodyHandle, double x, double y);

    public static native void rigidBodySetAngvel(long worldHandle, long bodyHandle, double angvel);

    public static native void rigidBodySetAdditionalMass(long worldHandle, long bodyHandle, double mass);

    public static native double rigidBodyGetLinearDamping(long worldHandle, long bodyHandle);

    public static native void rigidBodySetLinearDamping(long worldHandle, long bodyHandle, double damping);

    public static native double rigidBodyGetAngularDamping(long worldHandle, long bodyHandle);

    public static native void rigidBodySetAngularDamping(long worldHandle, long bodyHandle, double damping);

    public static native void rigidBodyAddForce(long worldHandle, long bodyHandle, double fx, double fy, boolean wake);

    public static native void rigidBodyAddForceAtPoint(
            long worldHandle, long bodyHandle, double fx, double fy, double px, double py, boolean wake);

    public static native void rigidBodyAddTorque(long worldHandle, long bodyHandle, double torque, boolean wake);

    public static native int colliderGetShapeType(long worldHandle, long colliderHandle);

    public static native boolean colliderIsSensor(long worldHandle, long colliderHandle);

    public static native void colliderSetSensor(long worldHandle, long colliderHandle, boolean sensor);

    public static native double colliderGetDensity(long worldHandle, long colliderHandle);

    public static native void colliderSetDensity(long worldHandle, long colliderHandle, double density);

    public static native double colliderGetFriction(long worldHandle, long colliderHandle);

    public static native void colliderSetFriction(long worldHandle, long colliderHandle, double friction);

    public static native double colliderGetRestitution(long worldHandle, long colliderHandle);

    public static native void colliderSetRestitution(long worldHandle, long colliderHandle, double restitution);

    public static native int colliderGetRestitutionCombineRule(long worldHandle, long colliderHandle);

    public static native void colliderSetRestitutionCombineRule(long worldHandle, long colliderHandle, int rule);

    public static native int colliderGetActiveEvents(long worldHandle, long colliderHandle);

    public static native void colliderSetActiveEvents(long worldHandle, long colliderHandle, int events);

    public static native int colliderGetActiveHooks(long worldHandle, long colliderHandle);

    public static native void colliderSetActiveHooks(long worldHandle, long colliderHandle, int hooks);

    public static native void colliderSetContactForceEventThreshold(
            long worldHandle, long colliderHandle, double threshold);

    public static native int colliderGetGroupMemberships(long worldHandle, long colliderHandle);

    public static native int colliderGetGroupFilters(long worldHandle, long colliderHandle);

    public static native void colliderSetCollisionGroups(
            long worldHandle, long colliderHandle, int memberships, int filter);

    public static native boolean worldAreColliding(long worldHandle, long colliderA, long colliderB);

    public static native int worldGetCollisionEventCount(long worldHandle);

    public static native int worldDrainCollisionEvents(long worldHandle, long[] colliderPairs, int[] flags);

    public static native int worldGetContactForceEventCount(long worldHandle);

    public static native int worldDrainContactForceEvents(long worldHandle, long[] colliderPairs, double[] data);

    public static native long worldCastRay(
            long worldHandle,
            double ox,
            double oy,
            double dx,
            double dy,
            double maxToi,
            boolean solid,
            int memberships,
            int filter,
            int queryFlags);

    public static native long worldCastRayAndGetNormal(
            long worldHandle,
            double ox,
            double oy,
            double dx,
            double dy,
            double maxToi,
            boolean solid,
            int memberships,
            int filter,
            int queryFlags,
            double[] out);

    public static native int worldQueryAabb(
            long worldHandle,
            double minX,
            double minY,
            double maxX,
            double maxY,
            int memberships,
            int filter,
            int queryFlags,
            long[] out);

    public static native double worldComputeFreeFallY(double initialY, double gravityY, double time);
}
