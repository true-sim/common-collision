package com.truesim.common.collision;

import com.truesim.common.math.geometry.Aabb;
import com.truesim.common.math.spatial.Bvh;

/**
 * A shape made of triangles, such as a mesh or a heightfield. Queries find the triangles near a region, and collision
 * works on those triangles one at a time. Each triangle has an index and a material id.
 */
public sealed interface ConcaveShape extends Shape permits TriangleMeshShape, HeightfieldShape {

    /** Number of triangle indices: valid triangle indices are {@code 0} to {@code triangleCount() - 1}. */
    int triangleCount();

    /** Writes the triangle's corners a, b, c as nine doubles to {@code out[at..at + 8]}. */
    void triangle(int triangle, double[] out, int at);

    int material(int triangle);

    /**
     * The triangle's active edges as bits: bit 0 for edge a-b, bit 1 for b-c, bit 2 for c-a. An edge is active when it
     * is a real crease (convex and bent by more than 5 degrees) or on the boundary; contacts on inactive edges use the
     * face normal, so objects slide smoothly over the seams between triangles.
     */
    int activeEdges(int triangle);

    /**
     * Visits the triangles whose bounds overlap the local-space box; returns how many were visited. The visitor returns
     * {@code false} to stop.
     */
    int queryAabb(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, Bvh.Visitor visitor);

    default int queryAabb(Aabb localBox, Bvh.Visitor visitor) {
        return queryAabb(
            localBox.min().x(), localBox.min().y(), localBox.min().z(),
            localBox.max().x(), localBox.max().y(), localBox.max().z(), visitor);
    }
}
