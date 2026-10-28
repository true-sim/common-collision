package com.truesim.common.collision;

import com.truesim.common.math.Vec3;

/**
 * Mutable result of a shape cast: the first time of impact along the motion, the target's surface normal facing the
 * moving shape, the touching points on both, and for meshes and heightfields the triangle and its material. Reuse one
 * instance per thread so shape casts allocate nothing.
 */
public final class ShapeCastHit {

    /** {@link #triangle()}, {@link #material()} and {@link #collider()} when there is none. */
    public static final int NONE = -1;

    double distance = Double.POSITIVE_INFINITY;
    double pointX;
    double pointY;
    double pointZ;
    double shapePointX;
    double shapePointY;
    double shapePointZ;
    double normalX;
    double normalY;
    double normalZ;
    int triangle = NONE;
    int material = NONE;
    int collider = NONE;

    /**
     * Motion parameter of the impact: the shape touches the target (comes within 1e-6 of it) after moving by
     * {@code distance * direction}, so with a unit direction this is the distance travelled. 0 when the shape starts
     * touching or overlapping the target and moves into it; infinite after a miss.
     */
    public double distance() {
        return distance;
    }

    public boolean isHit() {
        return distance != Double.POSITIVE_INFINITY;
    }

    /** Point on the target's surface where the moved shape touches it. */
    public Vec3 point() {
        return Vec3.of(pointX, pointY, pointZ);
    }

    /**
     * Point on the moved shape's surface where it touches the target: within 1e-6 of {@link #point()}, unless the shape
     * started inside the target.
     */
    public Vec3 shapePoint() {
        return Vec3.of(shapePointX, shapePointY, shapePointZ);
    }

    /**
     * Unit surface normal of the target at the impact, facing the moving shape. For a shape that starts overlapping the
     * target, the direction that separates them fastest.
     */
    public Vec3 normal() {
        return Vec3.of(normalX, normalY, normalZ);
    }

    public double pointX() {
        return pointX;
    }

    public double pointY() {
        return pointY;
    }

    public double pointZ() {
        return pointZ;
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

    /** The collider hit, for shape casts through a {@link CollisionWorld}; otherwise {@link #NONE}. */
    public int collider() {
        return collider;
    }

    void copyFrom(ShapeCastHit other) {
        distance = other.distance;
        pointX = other.pointX;
        pointY = other.pointY;
        pointZ = other.pointZ;
        shapePointX = other.shapePointX;
        shapePointY = other.shapePointY;
        shapePointZ = other.shapePointZ;
        normalX = other.normalX;
        normalY = other.normalY;
        normalZ = other.normalZ;
        triangle = other.triangle;
        material = other.material;
        collider = other.collider;
    }
}
