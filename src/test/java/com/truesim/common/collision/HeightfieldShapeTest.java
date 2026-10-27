package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;
import com.truesim.common.math.geometry.Triangle;
import com.truesim.common.math.random.Rng;
import java.util.Set;
import java.util.TreeSet;
import org.junit.jupiter.api.Test;

class HeightfieldShapeTest {

    /** Rolling hills of n x m cells, 2 by 1.5 units each, with material = (x + z) % 3. */
    static HeightfieldShape hills(int n, int m) {
        double[] heights = new double[(n + 1) * (m + 1)];
        for (int z = 0; z <= m; z++) {
            for (int x = 0; x <= n; x++) {
                heights[z * (n + 1) + x] = 2 * Math.sin(0.4 * x) * Math.cos(0.3 * z);
            }
        }
        int[] materials = new int[n * m];
        for (int z = 0; z < m; z++) {
            for (int x = 0; x < n; x++) {
                materials[z * n + x] = (x + z) % 3;
            }
        }
        return HeightfieldShape.of(n + 1, m + 1, 2, 1.5, heights, materials);
    }

    @Test
    void layoutAndAccessors() {
        HeightfieldShape field = hills(4, 3);
        assertThat(field.samplesX()).isEqualTo(5);
        assertThat(field.samplesZ()).isEqualTo(4);
        assertThat(field.cellsX()).isEqualTo(4);
        assertThat(field.cellsZ()).isEqualTo(3);
        assertThat(field.cellSizeX()).isEqualTo(2);
        assertThat(field.cellSizeZ()).isEqualTo(1.5);
        assertThat(field.triangleCount()).isEqualTo(24);
        assertThat(field.height(2, 1)).isEqualTo(2 * Math.sin(0.8) * Math.cos(0.3));
        assertThat(field.cellMaterial(2, 1)).isEqualTo(0);
        // Cell (2, 1) is cell 6, holding triangles 12 and 13.
        assertThat(field.material(12)).isEqualTo(field.cellMaterial(2, 1));
        assertThat(field.material(13)).isEqualTo(field.cellMaterial(2, 1));
        assertThat(field.triangle(12)).isEqualTo(new Triangle(
            Vec3.of(4, field.height(2, 1), 1.5), Vec3.of(4, field.height(2, 2), 3), Vec3.of(6, field.height(3, 1), 1.5)));
        assertThat(field.triangle(13)).isEqualTo(new Triangle(
            Vec3.of(6, field.height(3, 1), 1.5), Vec3.of(4, field.height(2, 2), 3), Vec3.of(6, field.height(3, 2), 3)));
        Aabb bounds = field.localBounds();
        assertThat(bounds.min().x()).isZero();
        assertThat(bounds.max().x()).isEqualTo(8);
        assertThat(bounds.max().z()).isEqualTo(4.5);
        for (int t = 0; t < field.triangleCount(); t++) {
            assertThat(field.triangle(t).bounds().min().y()).isGreaterThanOrEqualTo(bounds.min().y());
            assertThat(field.triangle(t).bounds().max().y()).isLessThanOrEqualTo(bounds.max().y());
            // Both triangles of a cell face up.
            assertThat(field.triangle(t).normal().y()).isPositive();
        }
        assertThat(HeightfieldShape.of(2, 2, 1, 1, new double[4]).material(1)).isZero();
    }

    @Test
    void queriesMatchBruteForce() {
        HeightfieldShape field = hills(20, 16);
        Rng rng = Rng.seeded(9);
        for (int i = 0; i < 400; i++) {
            Vec3 center = Vec3.of(rng.nextDouble(-4, 44), rng.nextDouble(-3, 3), rng.nextDouble(-4, 28));
            Vec3 half = Vec3.of(rng.nextDouble(0, 4), rng.nextDouble(0, 1), rng.nextDouble(0, 4));
            if (i % 4 == 0) {
                // Snap onto cell edges, where boxes touch the cells on both sides.
                center = Vec3.of(2 * Math.round(center.x() / 2), center.y(), 1.5 * Math.round(center.z() / 1.5));
                half = Vec3.of(2 * Math.round(half.x() / 2), half.y(), 1.5 * Math.round(half.z() / 1.5));
            }
            assertMatchesBruteForce(field, Aabb.fromCenterExtents(center, half));
        }
        assertMatchesBruteForce(field, field.localBounds());
        assertMatchesBruteForce(field, Aabb.fromCenterExtents(Vec3.of(-5, 0, 5), Vec3.ONE));
        assertMatchesBruteForce(field, Aabb.fromCenterExtents(Vec3.of(5, 10, 5), Vec3.ONE));
    }

    private static void assertMatchesBruteForce(HeightfieldShape field, Aabb box) {
        Set<Integer> expected = new TreeSet<>();
        for (int t = 0; t < field.triangleCount(); t++) {
            if (field.triangle(t).bounds().intersects(box)) {
                expected.add(t);
            }
        }
        Set<Integer> found = new TreeSet<>();
        int count = field.queryAabb(box, triangle -> found.add(triangle));
        assertThat(found).as("box %s", box).isEqualTo(expected);
        assertThat(count).isEqualTo(expected.size());
    }

    @Test
    void visitorCanStop() {
        HeightfieldShape flat = HeightfieldShape.of(3, 3, 1, 1, new double[9]);
        Aabb all = Aabb.fromCenterExtents(Vec3.of(1, 0, 1), Vec3.of(2, 1, 2));
        assertThat(flat.queryAabb(all, triangle -> false)).isEqualTo(1);
        assertThat(flat.queryAabb(all, triangle -> triangle == 0)).isEqualTo(2);
    }

    @Test
    void inputIsValidatedAndCopied() {
        assertThatIllegalArgumentException().isThrownBy(() -> HeightfieldShape.of(1, 2, 1, 1, new double[2]));
        assertThatIllegalArgumentException().isThrownBy(() -> HeightfieldShape.of(2, 1, 1, 1, new double[2]));
        assertThatIllegalArgumentException().isThrownBy(() -> HeightfieldShape.of(0, 0, 1, 1, new double[0]));
        assertThatIllegalArgumentException().isThrownBy(() -> HeightfieldShape.of(2, 2, 0, 1, new double[4]));
        assertThatIllegalArgumentException().isThrownBy(() -> HeightfieldShape.of(2, 2, 1, Double.NaN, new double[4]));
        assertThatIllegalArgumentException().isThrownBy(() -> HeightfieldShape.of(2, 2, 1, 1, new double[3]));
        assertThatIllegalArgumentException()
            .isThrownBy(() -> HeightfieldShape.of(2, 2, 1, 1, new double[4], new int[2]));
        assertThatIllegalArgumentException()
            .isThrownBy(() -> HeightfieldShape.of(2, 2, 1, 1, new double[] {0, 0, Double.NaN, 0}));
        assertThatIllegalArgumentException()
            .isThrownBy(() -> HeightfieldShape.of(2, 2, 1, 1, new double[] {0, Double.POSITIVE_INFINITY, 0, 0}));
        double[] heights = {1, 2, 3, 4};
        int[] materials = {5};
        HeightfieldShape field = HeightfieldShape.of(2, 2, 1, 1, heights, materials);
        heights[0] = 9;
        materials[0] = 9;
        assertThat(field.height(0, 0)).isEqualTo(1);
        assertThat(field.cellMaterial(0, 0)).isEqualTo(5);
    }
}
