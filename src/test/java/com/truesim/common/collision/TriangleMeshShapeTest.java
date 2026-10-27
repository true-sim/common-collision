package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;
import com.truesim.common.math.geometry.Triangle;
import com.truesim.common.math.random.Rng;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import org.junit.jupiter.api.Test;

class TriangleMeshShapeTest {

    /** A bumpy n x n grid of quads, two triangles each, with material = quadrant. */
    static TriangleMeshShape grid(int n) {
        List<Vec3> vertices = new ArrayList<>();
        for (int z = 0; z <= n; z++) {
            for (int x = 0; x <= n; x++) {
                vertices.add(Vec3.of(x, 0.3 * Math.sin(x) * Math.cos(z), z));
            }
        }
        int[] indices = new int[6 * n * n];
        int[] materials = new int[2 * n * n];
        int t = 0;
        for (int z = 0; z < n; z++) {
            for (int x = 0; x < n; x++) {
                int v = z * (n + 1) + x;
                int quadrant = (x < n / 2 ? 0 : 1) + (z < n / 2 ? 0 : 2);
                indices[3 * t] = v;
                indices[3 * t + 1] = v + n + 1;
                indices[3 * t + 2] = v + 1;
                materials[t++] = quadrant;
                indices[3 * t] = v + 1;
                indices[3 * t + 1] = v + n + 1;
                indices[3 * t + 2] = v + n + 2;
                materials[t++] = quadrant;
            }
        }
        return TriangleMeshShape.of(vertices, indices, materials);
    }

    @Test
    void accessors() {
        TriangleMeshShape mesh = grid(4);
        assertThat(mesh.vertexCount()).isEqualTo(25);
        assertThat(mesh.triangleCount()).isEqualTo(32);
        assertThat(mesh.vertexIndex(0, 0)).isZero();
        assertThat(mesh.vertexIndex(0, 1)).isEqualTo(5);
        assertThat(mesh.vertexIndex(0, 2)).isEqualTo(1);
        assertThat(mesh.vertex(6)).isEqualTo(Vec3.of(1, 0.3 * Math.sin(1) * Math.cos(1), 1));
        Triangle triangle = mesh.triangle(1);
        assertThat(triangle).isEqualTo(new Triangle(mesh.vertex(1), mesh.vertex(5), mesh.vertex(6)));
        double[] out = new double[10];
        mesh.triangle(1, out, 1);
        assertThat(out).containsExactly(0, 1, triangle.a().y(), 0, 0, triangle.b().y(), 1, 1, triangle.c().y(), 1);
        assertThat(mesh.material(0)).isZero();
        assertThat(mesh.material(31)).isEqualTo(3);
        Aabb bounds = mesh.localBounds();
        assertThat(bounds.min().x()).isZero();
        assertThat(bounds.max().z()).isEqualTo(4);
        assertThat(TriangleMeshShape.of(List.of(Vec3.ZERO, Vec3.ONE, Vec3.UP), new int[] {0, 1, 2}).material(0)).isZero();
    }

    @Test
    void queriesMatchBruteForce() {
        TriangleMeshShape mesh = grid(16);
        Rng rng = Rng.seeded(5);
        for (int i = 0; i < 200; i++) {
            Vec3 center = Vec3.of(rng.nextDouble(-2, 18), rng.nextDouble(-1, 1), rng.nextDouble(-2, 18));
            Vec3 half = Vec3.of(rng.nextDouble(0, 3), rng.nextDouble(0, 1), rng.nextDouble(0, 3));
            Aabb box = Aabb.fromCenterExtents(center, half);
            Set<Integer> expected = new TreeSet<>();
            for (int t = 0; t < mesh.triangleCount(); t++) {
                if (mesh.triangle(t).bounds().intersects(box)) {
                    expected.add(t);
                }
            }
            Set<Integer> found = new TreeSet<>();
            int count = mesh.queryAabb(box, triangle -> found.add(triangle));
            assertThat(found).isEqualTo(expected);
            assertThat(count).isEqualTo(expected.size());
        }
    }

    @Test
    void inputIsValidatedAndCopied() {
        List<Vec3> v = List.of(Vec3.ZERO, Vec3.ONE, Vec3.UP);
        assertThatIllegalArgumentException().isThrownBy(() -> TriangleMeshShape.of(List.of(), new int[] {0, 1, 2}));
        assertThatIllegalArgumentException().isThrownBy(() -> TriangleMeshShape.of(v, new int[0]));
        assertThatIllegalArgumentException().isThrownBy(() -> TriangleMeshShape.of(v, new int[] {0, 1}));
        assertThatIllegalArgumentException().isThrownBy(() -> TriangleMeshShape.of(v, new int[] {0, 1, 3}));
        assertThatIllegalArgumentException().isThrownBy(() -> TriangleMeshShape.of(v, new int[] {0, -1, 2}));
        assertThatIllegalArgumentException()
            .isThrownBy(() -> TriangleMeshShape.of(v, new int[] {0, 1, 2}, new int[] {1, 2}));
        int[] indices = {0, 1, 2};
        int[] materials = {7};
        TriangleMeshShape mesh = TriangleMeshShape.of(v, indices, materials);
        indices[0] = 2;
        materials[0] = 8;
        assertThat(mesh.vertexIndex(0, 0)).isZero();
        assertThat(mesh.material(0)).isEqualTo(7);
    }
}
