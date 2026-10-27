package com.truesim.common.collision;

import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;

/**
 * One triangle of a mesh or heightfield as a convex shape, so the narrow phase can collide it like any other. Mutable
 * and reused: the narrow phase loads each triangle's corners in turn.
 */
final class TriangleCore implements ConvexShape {

    /** Corners a, b, c as nine doubles. */
    final double[] corners = new double[9];

    @Override
    public Aabb localBounds() {
        return Aabb.EMPTY.include(Vec3.of(corners[0], corners[1], corners[2]))
            .include(Vec3.of(corners[3], corners[4], corners[5]))
            .include(Vec3.of(corners[6], corners[7], corners[8]));
    }

    @Override
    public double margin() {
        return 0;
    }

    @Override
    public void coreSupport(double dx, double dy, double dz, double[] out, int at) {
        double da = dx * corners[0] + dy * corners[1] + dz * corners[2];
        double db = dx * corners[3] + dy * corners[4] + dz * corners[5];
        double dc = dx * corners[6] + dy * corners[7] + dz * corners[8];
        int best = da >= db ? (da >= dc ? 0 : 6) : (db >= dc ? 3 : 6);
        out[at] = corners[best];
        out[at + 1] = corners[best + 1];
        out[at + 2] = corners[best + 2];
    }
}
