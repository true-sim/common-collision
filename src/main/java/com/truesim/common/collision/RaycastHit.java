package com.truesim.common.collision;

import com.truesim.common.math.Vec3;

/**
 * Mutable result of a raycast: the ray parameter, the hit point, the surface normal facing the ray, and for meshes and
 * heightfields the triangle and its material. Reuse one instance per thread so raycasts allocate nothing.
 */
public final class RaycastHit {

    /** {@link #triangle()} and {@link #material()} of a hit on a convex shape, which has no triangles. */
    public static final int NONE = -1;

    double distance = Double.POSITIVE_INFINITY;
    double pointX;
    double pointY;
    double pointZ;
    double normalX;
    double normalY;
    double normalZ;
    int triangle = NONE;
    int material = NONE;

    /**
     * Ray parameter of the hit: the hit point is {@code origin + distance * direction}, so with a unit direction this
     * is the distance. 0 when the ray starts inside a convex shape; infinite after a miss.
     */
    public double distance() {
        return distance;
    }

    public boolean isHit() {
        return distance != Double.POSITIVE_INFINITY;
    }

    public Vec3 point() {
        return Vec3.of(pointX, pointY, pointZ);
    }

    /** Unit surface normal at the hit, facing against the ray; minus the ray direction for a ray starting inside. */
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
}
