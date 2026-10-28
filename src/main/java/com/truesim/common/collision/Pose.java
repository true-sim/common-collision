package com.truesim.common.collision;

import com.truesim.common.math.Transform;

/** A convex shape placed by a transform, unpacked into doubles for the support function. */
final class Pose {

    private ConvexShape shape;
    private double tx;
    private double ty;
    private double tz;
    private double qx;
    private double qy;
    private double qz;
    private double qw;
    private double sx;
    private double sy;
    private double sz;
    private boolean uniform;
    /** World-space margin; 0 under non-uniform scale, where the margin is part of the scaled local support. */
    double margin;
    double centerX;
    double centerY;
    double centerZ;
    private final double[] local = new double[3];

    void set(ConvexShape shape, Transform transform) {
        set(shape, transform.translation().x(), transform.translation().y(), transform.translation().z(),
            transform.rotation().x(), transform.rotation().y(), transform.rotation().z(), transform.rotation().w(),
            transform.scale().x(), transform.scale().y(), transform.scale().z());
    }

    void set(ConvexShape shape, double tx, double ty, double tz, double qx, double qy, double qz, double qw,
        double sx, double sy, double sz) {
        this.shape = shape;
        this.tx = tx;
        this.ty = ty;
        this.tz = tz;
        this.qx = qx;
        this.qy = qy;
        this.qz = qz;
        this.qw = qw;
        this.sx = sx;
        this.sy = sy;
        this.sz = sz;
        uniform = sx == sy && sy == sz;
        margin = uniform ? shape.margin() * Math.abs(sx) : 0;
        centerX = tx;
        centerY = ty;
        centerZ = tz;
    }

    /** Moves the shape to translation ({@code x}, {@code y}, {@code z}), keeping its rotation and scale. */
    void moveTo(double x, double y, double z) {
        tx = x;
        ty = y;
        tz = z;
        centerX = x;
        centerY = y;
        centerZ = z;
    }

    /** Writes the world-space core point furthest in world direction d to {@code out[at..at + 2]}. */
    void coreSupport(double dx, double dy, double dz, double[] out, int at) {
        // Local direction: scale times inverse rotation of d (scale is diagonal, so its transpose is itself).
        double ux = 2 * (-qy * dz + qz * dy);
        double uy = 2 * (-qz * dx + qx * dz);
        double uz = 2 * (-qx * dy + qy * dx);
        double lx = (dx + qw * ux + (-qy * uz + qz * uy)) * sx;
        double ly = (dy + qw * uy + (-qz * ux + qx * uz)) * sy;
        double lz = (dz + qw * uz + (-qx * uy + qy * ux)) * sz;
        shape.coreSupport(lx, ly, lz, local, 0);
        double px = local[0];
        double py = local[1];
        double pz = local[2];
        if (!uniform) {
            // Under non-uniform scale a round shape becomes an ellipsoid: include the margin in local space.
            double length = Math.sqrt(lx * lx + ly * ly + lz * lz);
            double f = shape.margin() / length;
            px += lx * f;
            py += ly * f;
            pz += lz * f;
        }
        px *= sx;
        py *= sy;
        pz *= sz;
        double vx = 2 * (qy * pz - qz * py);
        double vy = 2 * (qz * px - qx * pz);
        double vz = 2 * (qx * py - qy * px);
        out[at] = tx + px + qw * vx + (qy * vz - qz * vy);
        out[at + 1] = ty + py + qw * vy + (qz * vx - qx * vz);
        out[at + 2] = tz + pz + qw * vz + (qx * vy - qy * vx);
    }
}
