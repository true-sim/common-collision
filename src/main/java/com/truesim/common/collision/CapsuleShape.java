package com.truesim.common.collision;

import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;

/**
 * Capsule along the local Y axis: the segment from {@code (0, -halfLength, 0)} to {@code (0, halfLength, 0)} with the
 * radius as margin.
 */
public record CapsuleShape(double halfLength, double radius) implements ConvexShape {

    public CapsuleShape {
        if (!(halfLength >= 0 && radius > 0)) {
            throw new IllegalArgumentException("Need halfLength >= 0 and radius > 0: " + halfLength + ", " + radius);
        }
    }

    @Override
    public Aabb localBounds() {
        return Aabb.fromCenterExtents(Vec3.ZERO, Vec3.of(radius, halfLength + radius, radius));
    }

    @Override
    public double margin() {
        return radius;
    }

    @Override
    public void coreSupport(double dx, double dy, double dz, double[] out, int at) {
        out[at] = 0;
        out[at + 1] = dy >= 0 ? halfLength : -halfLength;
        out[at + 2] = 0;
    }
}
