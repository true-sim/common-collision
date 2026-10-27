package com.truesim.common.collision;

import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;

/** Box centered on the local origin. */
public record BoxShape(Vec3 halfExtents) implements ConvexShape {

    public BoxShape {
        if (!(halfExtents.x() > 0 && halfExtents.y() > 0 && halfExtents.z() > 0)) {
            throw new IllegalArgumentException("Half extents must be positive: " + halfExtents);
        }
    }

    @Override
    public Aabb localBounds() {
        return Aabb.fromCenterExtents(Vec3.ZERO, halfExtents);
    }

    @Override
    public double margin() {
        return 0;
    }

    @Override
    public void coreSupport(double dx, double dy, double dz, double[] out, int at) {
        out[at] = dx >= 0 ? halfExtents.x() : -halfExtents.x();
        out[at + 1] = dy >= 0 ? halfExtents.y() : -halfExtents.y();
        out[at + 2] = dz >= 0 ? halfExtents.z() : -halfExtents.z();
    }
}
