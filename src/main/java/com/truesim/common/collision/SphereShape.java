package com.truesim.common.collision;

import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;

/** Sphere around the local origin: a point core with the radius as margin. */
public record SphereShape(double radius) implements ConvexShape {

    public SphereShape {
        if (!(radius > 0)) {
            throw new IllegalArgumentException("Radius must be positive: " + radius);
        }
    }

    @Override
    public Aabb localBounds() {
        return Aabb.fromCenterExtents(Vec3.ZERO, Vec3.splat(radius));
    }

    @Override
    public double margin() {
        return radius;
    }

    @Override
    public void coreSupport(double dx, double dy, double dz, double[] out, int at) {
        out[at] = 0;
        out[at + 1] = 0;
        out[at + 2] = 0;
    }
}
