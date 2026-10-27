package com.truesim.common.collision;

/**
 * A convex shape, described by its support function: the point furthest in a given direction. Round shapes are a
 * core (a point or segment) plus a {@link #margin()}, which keeps GJK exact and fast for them.
 */
public sealed interface ConvexShape extends Shape permits SphereShape, CapsuleShape, BoxShape, ConvexHullShape, TriangleCore {

    /** Radius added around the core: the sphere or capsule radius, 0 for sharp shapes. */
    double margin();

    /**
     * Writes the core point furthest in local direction ({@code dx}, {@code dy}, {@code dz}) to {@code out[at]},
     * {@code out[at + 1]}, {@code out[at + 2]}. The direction does not need to be normalized.
     */
    void coreSupport(double dx, double dy, double dz, double[] out, int at);
}
