package com.truesim.common.collision;

import com.truesim.common.math.Vec3;

/**
 * Mutable result of a narrow-phase query between shapes A and B: their signed distance, the contact normal (unit,
 * pointing from A to B) and the closest (or deepest) points on each surface. Reuse one instance per thread so queries
 * allocate nothing.
 */
public final class Contact {

    double distance;
    double normalX;
    double normalY;
    double normalZ;
    double ax;
    double ay;
    double az;
    double bx;
    double by;
    double bz;

    /** Distance between the surfaces: positive when apart, negative when penetrating (minus the depth). */
    public double distance() {
        return distance;
    }

    /** Penetration depth: {@code max(0, -distance())}. */
    public double depth() {
        return Math.max(0, -distance);
    }

    public boolean isPenetrating() {
        return distance < 0;
    }

    /** Unit normal from A to B: moving B along it by {@link #depth()} separates the shapes. */
    public Vec3 normal() {
        return new Vec3(normalX, normalY, normalZ);
    }

    /** Point on A's surface: the closest point to B, or A's deepest point inside B. */
    public Vec3 pointA() {
        return new Vec3(ax, ay, az);
    }

    /** Point on B's surface: the closest point to A, or B's deepest point inside A. */
    public Vec3 pointB() {
        return new Vec3(bx, by, bz);
    }

    public double normalX() {
        return normalX;
    }

    public double normalY() {
        return normalY;
    }

    public double normalZ() {
        return normalZ;
    }
}
