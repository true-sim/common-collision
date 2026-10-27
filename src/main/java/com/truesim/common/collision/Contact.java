package com.truesim.common.collision;

import com.truesim.common.math.Vec3;

/**
 * Mutable result of a narrow-phase query between shapes A and B: their signed distance, the contact normal (unit,
 * pointing from A to B) and the closest (or deepest) points on each surface. Reuse one instance per thread so queries
 * allocate nothing.
 */
public final class Contact {

    /** {@link #featureA()} and friends for a convex shape, which has no triangles. */
    public static final int NONE = -1;

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
    int featureA = NONE;
    int featureB = NONE;
    int materialA = NONE;
    int materialB = NONE;

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

    /** Triangle of shape A in contact (for a mesh or heightfield), or {@link #NONE}. */
    public int featureA() {
        return featureA;
    }

    /** Triangle of shape B in contact (for a mesh or heightfield), or {@link #NONE}. */
    public int featureB() {
        return featureB;
    }

    /** Material id of A's triangle in contact, or {@link #NONE}. */
    public int materialA() {
        return materialA;
    }

    /** Material id of B's triangle in contact, or {@link #NONE}. */
    public int materialB() {
        return materialB;
    }

    /** Copies {@code other} seen from the other side: A and B swapped, the normal reversed. */
    void copyFlipped(Contact other) {
        distance = other.distance;
        normalX = -other.normalX;
        normalY = -other.normalY;
        normalZ = -other.normalZ;
        ax = other.bx;
        ay = other.by;
        az = other.bz;
        bx = other.ax;
        by = other.ay;
        bz = other.az;
        featureA = other.featureB;
        featureB = other.featureA;
        materialA = other.materialB;
        materialB = other.materialA;
    }

    void copyFrom(Contact other) {
        distance = other.distance;
        normalX = other.normalX;
        normalY = other.normalY;
        normalZ = other.normalZ;
        ax = other.ax;
        ay = other.ay;
        az = other.az;
        bx = other.bx;
        by = other.by;
        bz = other.bz;
        featureA = other.featureA;
        featureB = other.featureB;
        materialA = other.materialA;
        materialB = other.materialB;
    }
}
