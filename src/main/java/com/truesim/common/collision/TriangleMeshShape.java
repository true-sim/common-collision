package com.truesim.common.collision;

import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;
import com.truesim.common.math.geometry.AabbArray;
import com.truesim.common.math.geometry.Triangle;
import com.truesim.common.math.spatial.Bvh;
import java.util.Arrays;
import java.util.List;

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
    public Aabb localBounds() {
        return bvh.bounds();
    }

    @Override
    public int queryAabb(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, Bvh.Visitor visitor) {
        return bvh.queryAabb(minX, minY, minZ, maxX, maxY, maxZ, visitor);
    }
}
