package com.truesim.common.collision;

import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;
import java.util.List;

/**
 * Convex hull of a set of points, e.g. a simplified car body. Points inside the hull are harmless; the support
 * function scans all points, so keep hulls small (tens of points).
 */
public final class ConvexHullShape implements ConvexShape {

    private final double[] points;
    private final Aabb bounds;

    private ConvexHullShape(double[] points, Aabb bounds) {
        this.points = points;
        this.bounds = bounds;
    }

    public static ConvexHullShape of(List<Vec3> points) {
        if (points.isEmpty()) {
            throw new IllegalArgumentException("A hull needs at least one point");
        }
        double[] coordinates = new double[3 * points.size()];
        Aabb bounds = Aabb.EMPTY;
        for (int i = 0; i < points.size(); i++) {
            Vec3 p = points.get(i);
            coordinates[3 * i] = p.x();
            coordinates[3 * i + 1] = p.y();
            coordinates[3 * i + 2] = p.z();
            bounds = bounds.include(p);
        }
        return new ConvexHullShape(coordinates, bounds);
    }

    public int pointCount() {
        return points.length / 3;
    }

    public Vec3 point(int index) {
        return new Vec3(points[3 * index], points[3 * index + 1], points[3 * index + 2]);
    }

    @Override
    public Aabb localBounds() {
        return bounds;
    }

    @Override
    public double margin() {
        return 0;
    }

    @Override
    public void coreSupport(double dx, double dy, double dz, double[] out, int at) {
        int best = 0;
        double bestDot = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < points.length; i += 3) {
            double dot = points[i] * dx + points[i + 1] * dy + points[i + 2] * dz;
            if (dot > bestDot) {
                bestDot = dot;
                best = i;
            }
        }
        out[at] = points[best];
        out[at + 1] = points[best + 1];
        out[at + 2] = points[best + 2];
    }
}
