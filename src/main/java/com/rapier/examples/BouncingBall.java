package com.rapier.examples;

import com.rapier.NativeBindings;

/**
 * Functional API example showing gravity with damping and restitution-dependent bouncing.
 */
public final class BouncingBall {
    private static final double GRAVITY_Y = -9.81;
    private static final int ALL_GROUPS = 0xFFFF_FFFF;

    private BouncingBall() {}

    public static void main(String[] args) {
        gravityWithDamping();
        bouncingRestitutionDemo();
    }

    private static void gravityWithDamping() {
        long world = NativeBindings.worldCreate();
        try {
            NativeBindings.worldSetGravity(world, 0.0, GRAVITY_Y);

            long body = NativeBindings.worldCreateRigidBody(
                    world, NativeBindings.RIGID_BODY_DYNAMIC, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0);
            NativeBindings.worldCreateColliderBall(world, body, 0.5, 1.0, 0.5, 0.0, false, ALL_GROUPS, ALL_GROUPS);

            NativeBindings.rigidBodySetLinearDamping(world, body, 0.8);

            double dt = 1.0 / 60.0;
            int steps = 60;
            for (int i = 0; i < steps; i++) {
                NativeBindings.worldStep(world, dt, 1);
            }

            double expectedY = NativeBindings.worldComputeFreeFallY(10.0, GRAVITY_Y, dt * steps);
            double actualY = NativeBindings.rigidBodyGetTranslationY(world, body);

            System.out.printf("Free-fall Y ~= %.3f, damped Y = %.3f (damping reduces fall)%n", expectedY, actualY);

            if (actualY <= expectedY) {
                throw new IllegalStateException("Damping should reduce the fall distance.");
            }
        } finally {
            NativeBindings.worldDestroy(world);
        }
    }

    private static void bouncingRestitutionDemo() {
        long world = NativeBindings.worldCreate();
        try {
            NativeBindings.worldSetGravity(world, 0.0, GRAVITY_Y);

            long groundBody = NativeBindings.worldCreateRigidBody(
                    world, NativeBindings.RIGID_BODY_FIXED, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0);
            NativeBindings.worldCreateColliderCuboid(
                    world, groundBody, 10.0, 0.5, 1.0, 0.8, 0.0, false, ALL_GROUPS, ALL_GROUPS);

            BallSetup high = createBall(world, -2.0, 4.0, 0.9);
            BallSetup mid = createBall(world, 0.0, 4.0, 0.6);
            BallSetup low = createBall(world, 2.0, 4.0, 0.3);

            int steps = 600;
            double dt = 1.0 / 60.0;
            for (int i = 0; i < steps; i++) {
                NativeBindings.worldStep(world, dt, 1);
                updateBounceCount(world, high);
                updateBounceCount(world, mid);
                updateBounceCount(world, low);
            }

            System.out.printf("Bounce counts -> high: %d, mid: %d, low: %d%n", high.bounces, mid.bounces, low.bounces);

            if (high.bounces < 3) {
                throw new IllegalStateException("High restitution should bounce at least 3 times.");
            }
            if (low.bounces >= mid.bounces || mid.bounces >= high.bounces) {
                throw new IllegalStateException("Lower restitution should bounce less.");
            }
        } finally {
            NativeBindings.worldDestroy(world);
        }
    }

    private static BallSetup createBall(long world, double x, double y, double restitution) {
        long body =
                NativeBindings.worldCreateRigidBody(world, NativeBindings.RIGID_BODY_DYNAMIC, x, y, 0.0, 0.0, 0.0, 0.0);
        NativeBindings.worldCreateColliderBall(world, body, 0.5, 1.0, 0.6, restitution, false, ALL_GROUPS, ALL_GROUPS);
        NativeBindings.rigidBodySetLinearDamping(world, body, 0.05);
        return new BallSetup(body, 0.5);
    }

    private static void updateBounceCount(long world, BallSetup ball) {
        double y = NativeBindings.rigidBodyGetTranslationY(world, ball.bodyHandle);
        double vy = NativeBindings.rigidBodyGetLinvelY(world, ball.bodyHandle);
        if (ball.prevVy < -0.1 && vy > 0.1 && y <= ball.radius + 0.05) {
            ball.bounces++;
        }
        ball.prevVy = vy;
    }

    private static final class BallSetup {
        private final long bodyHandle;
        private final double radius;
        private double prevVy;
        private int bounces;

        private BallSetup(long bodyHandle, double radius) {
            this.bodyHandle = bodyHandle;
            this.radius = radius;
        }
    }
}
