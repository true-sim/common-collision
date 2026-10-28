package com.truesim.common.collision;

import com.truesim.common.math.Vec3;

/**
 * Mutable result of a shape cast: how far the cast shape (A) got along its translation before its first impact, and the
 * contact with what it hit (B) there. Reuse one instance per thread so casts allocate nothing.
 *
 * <p>A cast stops just short of touching ({@link NarrowPhase#CAST_SEPARATION} apart), so at the reported position the
 * shapes are still separated and the normal and points are their closest ones. A shape already touching or
 * overlapping at the start hits at fraction 0, with {@link #distance()} negative when penetrating.
 */
public final class ShapeCastHit {

    /** {@link #triangle()}, {@link #material()} and {@link #collider()} when there is none. */
    public static final int NONE = -1;

    double fraction = Double.POSITIVE_INFINITY;
    double distance = Double.POSITIVE_INFINITY;
    double normalX;
    double normalY;
    double normalZ;
    double ax;
    double ay;
    double az;
    double bx;
    double by;
    double bz;
    int triangle = NONE;
    int material = NONE;
    int collider = NONE;

    /** Fraction of the translation moved before the impact, in [0, 1]; positive infinity without a hit. */
    public double fraction() {
        return fraction;
    }

    public boolean isHit() {
        return fraction != Double.POSITIVE_INFINITY;
    }

    /** Signed distance between the shapes at the impact: about {@link NarrowPhase#CAST_SEPARATION}, or less at the start. */
    public double distance() {
        return distance;
    }

    /** Unit contact normal from the cast shape (A) to what it hit (B), as in {@link Contact#normal()}. */
    public Vec3 normal() {
        return new Vec3(normalX, normalY, normalZ);
    }

    /** Point on the cast shape at the impact: the closest point to B, or its deepest point when starting inside. */
    public Vec3 pointA() {
        return new Vec3(ax, ay, az);
    }

    /** Point on what the cast shape hit: the closest point to A, or its deepest point when starting inside. */
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

    /** Index of the triangle hit in a mesh or heightfield, or {@link #NONE}. */
    public int triangle() {
        return triangle;
    }

    /** Material id of the triangle hit in a mesh or heightfield, or {@link #NONE}. */
    public int material() {
        return material;
    }

    /** The collider hit, for casts through a {@link CollisionWorld}; otherwise {@link #NONE}. */
    public int collider() {
        return collider;
    }

    void miss() {
        fraction = Double.POSITIVE_INFINITY;
        distance = Double.POSITIVE_INFINITY;
        triangle = NONE;
        material = NONE;
        collider = NONE;
    }

    /** Takes the impact at {@code fraction} from a contact between the cast shape (A) and what it hit (B). */
    void set(double fraction, Contact contact) {
        this.fraction = fraction;
        distance = contact.distance;
        normalX = contact.normalX;
        normalY = contact.normalY;
        normalZ = contact.normalZ;
        ax = contact.ax;
        ay = contact.ay;
        az = contact.az;
        bx = contact.bx;
        by = contact.by;
        bz = contact.bz;
        triangle = contact.featureB;
        material = contact.materialB;
        collider = NONE;
    }

    void copyFrom(ShapeCastHit other) {
        fraction = other.fraction;
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
        triangle = other.triangle;
        material = other.material;
        collider = other.collider;
    }
}
