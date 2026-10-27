package com.truesim.common.collision;

import com.truesim.annotations.AllowAllocation;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;
import com.truesim.common.math.geometry.AabbArray;
import com.truesim.common.math.geometry.Triangle;
import com.truesim.common.math.spatial.Bvh;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Triangle mesh for static or kinematic geometry such as a track, with a material id per triangle.
 *
 * <p>The mesh keeps its own {@link Bvh} over its triangles in local space, built once. Every collider that uses the
 * mesh shares it and adds only a transform: queries move the ray or shape into the mesh's local space and descend this
 * tree. The mesh is immutable; its queries are read-only and thread-safe.
 *
 * <pre>{@code
 * TriangleMeshShape track = TriangleMeshShape.of(vertices, indices, materials);
 * int found = track.queryAabb(localBox, triangle -> { collide(triangle, track.material(triangle)); return true; });
 * }</pre>
 */
public final class TriangleMeshShape implements ConcaveShape {

    private final double[] vertices;
    private final int[] indices;
    private final int[] materials;
    private final byte[] activeEdges;
    private final Bvh bvh;

    private TriangleMeshShape(double[] vertices, int[] indices, int[] materials) {
        this.vertices = vertices;
        this.indices = indices;
        this.materials = materials;
        int triangles = indices.length / 3;
        AabbArray bounds = AabbArray.onHeap(triangles);
        for (int t = 0; t < triangles; t++) {
            int a = 3 * indices[3 * t];
            int b = 3 * indices[3 * t + 1];
            int c = 3 * indices[3 * t + 2];
            bounds.set(t,
                Math.min(vertices[a], Math.min(vertices[b], vertices[c])),
                Math.min(vertices[a + 1], Math.min(vertices[b + 1], vertices[c + 1])),
                Math.min(vertices[a + 2], Math.min(vertices[b + 2], vertices[c + 2])),
                Math.max(vertices[a], Math.max(vertices[b], vertices[c])),
                Math.max(vertices[a + 1], Math.max(vertices[b + 1], vertices[c + 1])),
                Math.max(vertices[a + 2], Math.max(vertices[b + 2], vertices[c + 2])));
        }
        this.bvh = Bvh.build(bounds);
        this.activeEdges = activeEdges(vertices, indices);
    }

    /** Finds each edge's neighbor across it and classifies the edge; unshared, non-manifold or inconsistently wound edges stay active. */
    @AllowAllocation("one-time setup")
    private static byte[] activeEdges(double[] vertices, int[] indices) {
        int edges = indices.length;
        int[] partner = new int[edges];
        Arrays.fill(partner, -1);
        Map<Long, Integer> first = new HashMap<>();
        for (int edge = 0; edge < edges; edge++) {
            int a = indices[edge];
            int b = indices[next(edge)];
            Integer other = first.putIfAbsent(((long) Math.min(a, b) << 32) | Math.max(a, b), edge);
            if (other != null) {
                if (partner[other] == -1) {
                    partner[other] = edge;
                    partner[edge] = other;
                } else {
                    // Three or more triangles share the edge: non-manifold, keep it sharp.
                    if (partner[other] >= 0) {
                        partner[partner[other]] = -2;
                    }
                    partner[other] = -2;
                    partner[edge] = -2;
                }
            }
        }
        byte[] active = new byte[edges / 3];
        for (int edge = 0; edge < edges; edge++) {
            int other = partner[edge];
            int a = 3 * indices[edge];
            int b = 3 * indices[next(edge)];
            boolean sharp = other < 0 || indices[other] != indices[next(edge)];
            if (!sharp) {
                int c = 3 * indices[next(next(edge))];
                int d = 3 * indices[next(next(other))];
                sharp = Edges.active(vertices[a], vertices[a + 1], vertices[a + 2], vertices[b], vertices[b + 1],
                    vertices[b + 2], vertices[c], vertices[c + 1], vertices[c + 2], vertices[d], vertices[d + 1],
                    vertices[d + 2]);
            }
            if (sharp) {
                active[edge / 3] |= (byte) (1 << (edge % 3));
            }
        }
        return active;
    }

    /** The next edge (and corner) of the same triangle. */
    private static int next(int edge) {
        return edge % 3 == 2 ? edge - 2 : edge + 1;
    }

    /** A mesh with material id 0 on every triangle. */
    public static TriangleMeshShape of(List<Vec3> vertices, int[] indices) {
        return of(vertices, indices, new int[indices.length / 3]);
    }

    /**
     * A mesh from vertices and triangles: {@code indices} holds three vertex indices per triangle, {@code materials}
     * one material id per triangle. The arrays are copied.
     */
    public static TriangleMeshShape of(List<Vec3> vertices, int[] indices, int[] materials) {
        if (vertices.isEmpty() || indices.length == 0 || indices.length % 3 != 0) {
            throw new IllegalArgumentException(
                "Need vertices and three indices per triangle: " + vertices.size() + " vertices, " + indices.length
                    + " indices");
        }
        if (materials.length != indices.length / 3) {
            throw new IllegalArgumentException(
                "Need one material per triangle: " + materials.length + " for " + indices.length / 3 + " triangles");
        }
        for (int index : indices) {
            if (index < 0 || index >= vertices.size()) {
                throw new IllegalArgumentException("Vertex index out of range: " + index);
            }
        }
        double[] coordinates = new double[3 * vertices.size()];
        for (int i = 0; i < vertices.size(); i++) {
            Vec3 v = vertices.get(i);
            coordinates[3 * i] = v.x();
            coordinates[3 * i + 1] = v.y();
            coordinates[3 * i + 2] = v.z();
        }
        return new TriangleMeshShape(
            coordinates, Arrays.copyOf(indices, indices.length), Arrays.copyOf(materials, materials.length));
    }

    public int vertexCount() {
        return vertices.length / 3;
    }

    @Override
    public int triangleCount() {
        return indices.length / 3;
    }

    public Vec3 vertex(int index) {
        return Vec3.of(vertices[3 * index], vertices[3 * index + 1], vertices[3 * index + 2]);
    }

    /** Vertex index {@code corner} (0, 1 or 2) of triangle {@code triangle}. */
    public int vertexIndex(int triangle, int corner) {
        return indices[3 * triangle + corner];
    }

    public Triangle triangle(int triangle) {
        return new Triangle(
            vertex(indices[3 * triangle]), vertex(indices[3 * triangle + 1]), vertex(indices[3 * triangle + 2]));
    }

    @Override
    public void triangle(int triangle, double[] out, int at) {
        for (int corner = 0; corner < 3; corner++) {
            int v = 3 * indices[3 * triangle + corner];
            out[at + 3 * corner] = vertices[v];
            out[at + 3 * corner + 1] = vertices[v + 1];
            out[at + 3 * corner + 2] = vertices[v + 2];
        }
    }

    @Override
    public int material(int triangle) {
        return materials[triangle];
    }

    @Override
    public int activeEdges(int triangle) {
        return activeEdges[triangle];
    }

    @Override
    public Aabb localBounds() {
        return bvh.bounds();
    }

    boolean raycast(double ox, double oy, double oz, double dx, double dy, double dz, double max, Bvh.RayTest test,
        com.truesim.common.math.spatial.RayHit hit) {
        return bvh.raycast(ox, oy, oz, dx, dy, dz, max, test, hit);
    }

    @Override
    public int queryAabb(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, Bvh.Visitor visitor) {
        return bvh.queryAabb(minX, minY, minZ, maxX, maxY, maxZ, visitor);
    }
}
