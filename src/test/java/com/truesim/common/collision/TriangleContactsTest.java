package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.truesim.common.math.Quat;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.random.Rng;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class TriangleContactsTest {

    private static final double EPS = 1e-9;

    private final NarrowPhase narrow = new NarrowPhase();
    private final ContactBuffer contacts = new ContactBuffer(64);

    /** A flat n x n grid at y = 0, two triangles per unit cell, facing up. */
    private static TriangleMeshShape flat(int n) {
        HeightfieldShape field = HeightfieldShape.of(n + 1, n + 1, 1, 1, new double[(n + 1) * (n + 1)]);
        return meshOf(field);
    }

    /** The same triangles as a concave shape, as a mesh with shared vertices. */
    static TriangleMeshShape meshOf(ConcaveShape shape) {
        List<Vec3> vertices = new ArrayList<>();
        Map<Vec3, Integer> index = new HashMap<>();
        int[] indices = new int[3 * shape.triangleCount()];
        int[] materials = new int[shape.triangleCount()];
        double[] c = new double[9];
        for (int t = 0; t < shape.triangleCount(); t++) {
            shape.triangle(t, c, 0);
            for (int k = 0; k < 3; k++) {
                Vec3 v = Vec3.of(c[3 * k], c[3 * k + 1], c[3 * k + 2]);
                Integer i = index.get(v);
                if (i == null) {
                    i = vertices.size();
                    index.put(v, i);
                    vertices.add(v);
                }
                indices[3 * t + k] = i;
            }
            materials[t] = shape.material(t);
        }
        return TriangleMeshShape.of(vertices, indices, materials);
    }

    private static void assertNormal(Contact contact, double x, double y, double z, double tolerance) {
        assertThat(contact.normalX()).isCloseTo(x, within(tolerance));
        assertThat(contact.normalY()).isCloseTo(y, within(tolerance));
        assertThat(contact.normalZ()).isCloseTo(z, within(tolerance));
    }

    // ---------------------------------------------------------------- smooth surfaces

    @Test
    void boxRestsOnSeamsWithoutCatching() {
        TriangleMeshShape floor = flat(8);
        BoxShape box = new BoxShape(Vec3.of(0.5, 0.5, 0.5));
        // Centered on a vertex where six triangles meet, turned so its corners sit on edges and diagonals.
        Transform pose = Transform.of(Vec3.of(4, 0.45, 4), Quat.fromAxisAngle(Vec3.UP, 0.3));
        int count = narrow.contacts(box, pose, floor, Transform.IDENTITY, 0.01, contacts);
        assertThat(count).isGreaterThanOrEqualTo(6).isEqualTo(contacts.size());
        for (int i = 0; i < count; i++) {
            Contact contact = contacts.get(i);
            assertNormal(contact, 0, -1, 0, 1e-9);
            // Triangles reaching only partly under the box may measure less than the full 0.05 sideways.
            assertThat(contact.distance()).isBetween(-0.05 - 1e-9, 0.0);
            assertThat(contact.pointB().y()).isCloseTo(0, within(1e-9));
            assertThat(contact.featureA()).isEqualTo(Contact.NONE);
            assertThat(contact.materialA()).isEqualTo(Contact.NONE);
            assertThat(contact.featureB()).isBetween(0, floor.triangleCount() - 1);
            assertThat(contact.materialB()).isZero();
        }
    }

    @Test
    void sphereSlidesAcrossSeams() {
        TriangleMeshShape floor = flat(6);
        SphereShape ball = new SphereShape(0.5);
        Rng rng = Rng.seeded(12);
        for (int i = 0; i < 200; i++) {
            // Anywhere over the interior, including exactly over edges and vertices.
            double x = i % 4 == 0 ? Math.round(rng.nextDouble(1, 5)) : rng.nextDouble(1, 5);
            double z = i % 3 == 0 ? Math.round(rng.nextDouble(1, 5)) : rng.nextDouble(1, 5);
            int count = narrow.contacts(ball, Transform.ofTranslation(Vec3.of(x, 0.4, z)), floor, Transform.IDENTITY, 0,
                contacts);
            assertThat(count).isPositive();
            for (int k = 0; k < count; k++) {
                assertNormal(contacts.get(k), 0, -1, 0, 1e-9);
                // Neighbors touched only at an edge measure less, but push the same way.
                assertThat(contacts.get(k).distance()).isBetween(-0.1 - 1e-9, 0.0);
            }
            assertThat(contacts.deepest().distance()).isCloseTo(-0.1, within(1e-9));
        }
    }

    @Test
    void heightfieldsBehaveLikeMeshes() {
        HeightfieldShape hills = HeightfieldShapeTest.hills(12, 10);
        TriangleMeshShape mesh = meshOf(hills);
        for (int t = 0; t < hills.triangleCount(); t++) {
            assertThat(hills.activeEdges(t)).as("triangle %d", t).isEqualTo(mesh.activeEdges(t));
        }
        ContactBuffer other = new ContactBuffer(64);
        Rng rng = Rng.seeded(13);
        for (int i = 0; i < 200; i++) {
            Vec3 center = Vec3.of(rng.nextDouble(0, 24), rng.nextDouble(-1.5, 2.5), rng.nextDouble(0, 15));
            ConvexShape shape = i % 2 == 0 ? new SphereShape(0.8) : new BoxShape(Vec3.of(0.6, 0.4, 0.9));
            Transform pose = Transform.of(center, rng.nextRotation());
            int count = narrow.contacts(shape, pose, hills, Transform.IDENTITY, 0.05, contacts);
            assertThat(narrow.contacts(shape, pose, mesh, Transform.IDENTITY, 0.05, other)).isEqualTo(count);
            Map<Integer, Contact> byTriangle = new HashMap<>();
            for (int k = 0; k < count; k++) {
                byTriangle.put(other.get(k).featureB(), other.get(k));
            }
            for (int k = 0; k < count; k++) {
                Contact a = contacts.get(k);
                Contact b = byTriangle.get(a.featureB());
                assertThat(b).isNotNull();
                assertThat(a.distance()).isCloseTo(b.distance(), within(1e-9));
                assertNormal(a, b.normalX(), b.normalY(), b.normalZ(), 1e-9);
                assertThat(a.materialB()).isEqualTo(hills.material(a.featureB()));
            }
        }
    }

    @Test
    void flatHeightfieldEdges() {
        HeightfieldShape field = HeightfieldShape.of(4, 4, 1, 1, new double[16]);
        // Cell (1, 1) is surrounded by cells: its triangles have no active edges.
        assertThat(field.activeEdges(2 * 4)).isZero();
        assertThat(field.activeEdges(2 * 4 + 1)).isZero();
        // Corner cell (0, 0): even triangle edges a-b (x = 0) and c-a (z = 0) lie on the boundary.
        assertThat(field.activeEdges(0)).isEqualTo(0b101);
        // Corner cell (2, 2): odd triangle edges b-c (z = 3) and c-a (x = 3) lie on the boundary.
        assertThat(field.activeEdges(2 * 8 + 1)).isEqualTo(0b110);
    }

    // ---------------------------------------------------------------- creases

    @Test
    void convexRidgeKeepsItsEdgeNormal() {
        // A roof: two triangles meeting at a ridge along z at height 1, 45 degrees each side.
        TriangleMeshShape roof = TriangleMeshShape.of(
            List.of(Vec3.of(-1, 0, -2), Vec3.of(0, 1, -2), Vec3.of(0, 1, 2), Vec3.of(-1, 0, 2), Vec3.of(1, 0, -2),
                Vec3.of(1, 0, 2)),
            new int[] {0, 2, 1, 0, 3, 2, 1, 2, 5, 1, 5, 4});
        // The ridge edge (1-2) is active; the diagonals inside each slope are flat, so inactive.
        assertThat(roof.activeEdges(0) & 0b010).isNotZero();
        int count = narrow.contacts(new SphereShape(0.5), Transform.ofTranslation(Vec3.of(0, 1.4, 0)), roof,
            Transform.IDENTITY, 0, contacts);
        assertThat(count).isPositive();
        for (int k = 0; k < count; k++) {
            assertNormal(contacts.get(k), 0, -1, 0, 1e-9);
            assertThat(contacts.get(k).distance()).isCloseTo(-0.1, within(1e-9));
        }
    }

    @Test
    void concaveValleyUsesFaceNormals() {
        // A valley: the same roof upside down, crease at height 0.
        TriangleMeshShape valley = TriangleMeshShape.of(
            List.of(Vec3.of(-1, 1, -2), Vec3.of(0, 0, -2), Vec3.of(0, 0, 2), Vec3.of(-1, 1, 2), Vec3.of(1, 1, -2),
                Vec3.of(1, 1, 2)),
            new int[] {0, 2, 1, 0, 3, 2, 1, 2, 5, 1, 5, 4});
        assertThat(valley.activeEdges(0) & 0b010).isZero();
        // A small box sitting in the crease touches it with its bottom edge: contacts use the slopes' normals.
        int count = narrow.contacts(new BoxShape(Vec3.of(0.1, 0.1, 0.1)),
            Transform.of(Vec3.of(0, 0.1 * Math.sqrt(2) - 0.01, 0), Quat.fromAxisAngle(Vec3.of(0, 0, 1), Math.PI / 4)),
            valley, Transform.IDENTITY, 0, contacts);
        assertThat(count).isPositive();
        double s = Math.sqrt(0.5);
        for (int k = 0; k < count; k++) {
            Contact contact = contacts.get(k);
            assertThat(Math.abs(contact.normalX())).isCloseTo(s, within(1e-9));
            assertThat(contact.normalY()).isCloseTo(-s, within(1e-9));
        }
    }

    // ---------------------------------------------------------------- exact against brute force

    @Test
    void triangleSoupMatchesBruteForce() {
        // Disconnected triangles: every edge is a boundary, so contacts are the plain narrow-phase results.
        Rng rng = Rng.seeded(14);
        List<Vec3> vertices = new ArrayList<>();
        int[] indices = new int[3 * 150];
        int[] materials = new int[150];
        for (int t = 0; t < 150; t++) {
            Vec3 center = Vec3.of(rng.nextDouble(-8, 8), rng.nextDouble(-2, 2), rng.nextDouble(-8, 8));
            for (int k = 0; k < 3; k++) {
                indices[3 * t + k] = vertices.size();
                vertices.add(center.add(rng.nextVec3InSphere().scale(1.5)));
            }
            materials[t] = t % 5;
        }
        TriangleMeshShape soup = TriangleMeshShape.of(vertices, indices, materials);
        Transform placed = Transform.of(Vec3.of(1, 2, 3), Quat.fromYawPitchRoll(0.4, -0.2, 0.3), Vec3.of(1, 1.5, 0.8));
        ContactBuffer big = new ContactBuffer(200);
        Contact single = new Contact();
        for (int i = 0; i < 100; i++) {
            ConvexShape shape = switch (i % 3) {
                case 0 -> new SphereShape(1);
                case 1 -> new CapsuleShape(0.8, 0.4);
                default -> new BoxShape(Vec3.of(0.7, 0.5, 1));
            };
            Transform pose = Transform.of(placed.transformPoint(Vec3.of(rng.nextDouble(-8, 8), rng.nextDouble(-2, 2),
                rng.nextDouble(-8, 8))), rng.nextRotation());
            narrow.contacts(shape, pose, soup, placed, 0.2, big);
            Map<Integer, Double> found = new HashMap<>();
            for (int k = 0; k < big.size(); k++) {
                found.put(big.get(k).featureB(), big.get(k).distance());
                assertThat(big.get(k).materialB()).isEqualTo(big.get(k).featureB() % 5);
            }
            for (int t = 0; t < soup.triangleCount(); t++) {
                Triangle3 world = Triangle3.of(soup, t, placed);
                double distance = narrow.query(shape, pose, world.hull, Transform.IDENTITY, single);
                if (distance <= 0.2 - 1e-9) {
                    assertThat(found).containsKey(t);
                    assertThat(found.get(t)).isCloseTo(distance, within(1e-7));
                } else if (distance > 0.2 + 1e-9) {
                    assertThat(found).doesNotContainKey(t);
                }
            }
        }
    }

    /** A world-space triangle as a hull, for reference queries. */
    private record Triangle3(ConvexHullShape hull) {
        static Triangle3 of(ConcaveShape shape, int triangle, Transform transform) {
            double[] c = new double[9];
            shape.triangle(triangle, c, 0);
            return new Triangle3(ConvexHullShape.of(List.of(transform.transformPoint(Vec3.of(c[0], c[1], c[2])),
                transform.transformPoint(Vec3.of(c[3], c[4], c[5])), transform.transformPoint(Vec3.of(c[6], c[7], c[8])))));
        }
    }

    @Test
    void transformedMesh() {
        TriangleMeshShape floor = flat(6);
        // Tilted 30 degrees about z, stretched along x, moved.
        Quat tilt = Quat.fromAxisAngle(Vec3.of(0, 0, 1), Math.PI / 6);
        Transform pose = Transform.of(Vec3.of(-3, 1, -3), tilt, Vec3.of(2, 1, 1));
        Vec3 up = tilt.rotate(Vec3.UP);
        Vec3 onFloor = pose.transformPoint(Vec3.of(3, 0, 3));
        int count = narrow.contacts(new SphereShape(1), Transform.ofTranslation(onFloor.add(up.scale(0.9))), floor, pose,
            0, contacts);
        assertThat(count).isPositive();
        for (int k = 0; k < count; k++) {
            assertNormal(contacts.get(k), -up.x(), -up.y(), -up.z(), 1e-9);
            assertThat(contacts.get(k).distance()).isCloseTo(-0.1, within(1e-9));
        }
        // Out of reach: nothing.
        assertThat(narrow.contacts(new SphereShape(1), Transform.ofTranslation(onFloor.add(up.scale(3))), floor, pose,
            0.5, contacts)).isZero();
        assertThat(contacts.isEmpty()).isTrue();
        assertThat(contacts.deepest()).isNull();
    }

    @Test
    void negativeContactDistanceKeepsOnlyDeepContacts() {
        TriangleMeshShape floor = flat(4);
        Transform pose = Transform.ofTranslation(Vec3.of(2, 0.4, 2));
        assertThat(narrow.contacts(new SphereShape(0.5), pose, floor, Transform.IDENTITY, -0.2, contacts)).isZero();
        assertThat(narrow.contacts(new SphereShape(0.5), pose, floor, Transform.IDENTITY, -0.05, contacts)).isPositive();
    }

    // ---------------------------------------------------------------- buffer and edge cases

    @Test
    void fullBufferKeepsTheDeepestContacts() {
        TriangleMeshShape floor = flat(8);
        ContactBuffer two = new ContactBuffer(2);
        assertThat(two.capacity()).isEqualTo(2);
        // Tilted box: its lower corner digs deeper than the rest.
        Transform pose = Transform.of(Vec3.of(4, 0.5, 4), Quat.fromYawPitchRoll(0.2, 0.3, 0.25));
        BoxShape box = new BoxShape(Vec3.of(0.5, 0.5, 0.5));
        int all = narrow.contacts(box, pose, floor, Transform.IDENTITY, 0.5, contacts);
        assertThat(all).isGreaterThan(2);
        assertThat(narrow.contacts(box, pose, floor, Transform.IDENTITY, 0.5, two)).isEqualTo(2);
        assertThat(two.isEmpty()).isFalse();
        List<Double> distances = new ArrayList<>();
        for (int k = 0; k < all; k++) {
            distances.add(contacts.get(k).distance());
        }
        distances.sort(null);
        List<Double> kept = new ArrayList<>(List.of(two.get(0).distance(), two.get(1).distance()));
        kept.sort(null);
        assertThat(kept).containsExactly(distances.get(0), distances.get(1));
        assertThat(two.deepest().distance()).isEqualTo(distances.get(0));
        assertThat(contacts.deepest().distance()).isEqualTo(distances.get(0));
        assertThatThrownBy(() -> two.get(2)).isInstanceOf(IndexOutOfBoundsException.class);
        assertThatIllegalArgumentException().isThrownBy(() -> new ContactBuffer(0));
    }

    @Test
    void meshEdgeClassification() {
        List<Vec3> v = List.of(Vec3.of(0, 0, 0), Vec3.of(1, 0, 0), Vec3.of(0, 0, 1), Vec3.of(0, 1, 0), Vec3.of(0, -1, 0),
            Vec3.of(1, 1, 1), Vec3.of(1, 0, 1));
        // Three triangles on edge 0-1: non-manifold, all sharp.
        TriangleMeshShape fan = TriangleMeshShape.of(v, new int[] {0, 1, 2, 1, 0, 3, 1, 0, 4});
        for (int t = 0; t < 3; t++) {
            assertThat(fan.activeEdges(t) & 1).isEqualTo(1);
        }
        // Four triangles on it.
        TriangleMeshShape fan4 = TriangleMeshShape.of(v, new int[] {0, 1, 2, 1, 0, 3, 1, 0, 4, 0, 1, 5});
        for (int t = 0; t < 4; t++) {
            assertThat(fan4.activeEdges(t) & 1).isEqualTo(1);
        }
        // Two coplanar triangles wound inconsistently: the shared edge stays sharp.
        TriangleMeshShape flipped = TriangleMeshShape.of(v, new int[] {0, 1, 2, 1, 2, 6});
        assertThat(flipped.activeEdges(0) & 0b010).isNotZero();
        // Wound consistently: the flat seam is inactive.
        TriangleMeshShape square = TriangleMeshShape.of(v, new int[] {0, 2, 1, 1, 2, 6});
        assertThat(square.activeEdges(0) & 0b010).isZero();
        assertThat(square.activeEdges(1) & 0b001).isZero();
        // A degenerate triangle keeps all its edges sharp.
        TriangleMeshShape sliver = TriangleMeshShape.of(List.of(Vec3.ZERO, Vec3.UNIT_X, Vec3.of(2, 0, 0), Vec3.of(1, 0, 1)),
            new int[] {0, 1, 2, 1, 0, 3});
        assertThat(sliver.activeEdges(0)).isEqualTo(7);
    }

    @Test
    void convexQueriesHaveNoFeatures() {
        Contact contact = new Contact();
        narrow.query(new SphereShape(1), Transform.IDENTITY, new SphereShape(1), Transform.ofTranslation(Vec3.of(3, 0, 0)),
            contact);
        assertThat(contact.featureA()).isEqualTo(Contact.NONE);
        assertThat(contact.featureB()).isEqualTo(Contact.NONE);
        assertThat(contact.materialA()).isEqualTo(Contact.NONE);
        assertThat(contact.materialB()).isEqualTo(Contact.NONE);
    }

    @Test
    void triangleCoreBounds() {
        TriangleCore core = new TriangleCore();
        System.arraycopy(new double[] {0, 1, 2, -1, 3, 0, 2, 0, 1}, 0, core.corners, 0, 9);
        assertThat(core.localBounds().min()).isEqualTo(Vec3.of(-1, 0, 0));
        assertThat(core.localBounds().max()).isEqualTo(Vec3.of(2, 3, 2));
        assertThat(core.margin()).isZero();
        double[] out = new double[3];
        double[][] directions = {{0, -1, 1}, {-1, 1, 0}, {1, 0, 0}, {1, 0.6, 0}, {0, 0, -1}};
        int[] expected = {0, 3, 6, 6, 3};
        for (int i = 0; i < directions.length; i++) {
            core.coreSupport(directions[i][0], directions[i][1], directions[i][2], out, 0);
            assertThat(out).containsExactly(core.corners[expected[i]], core.corners[expected[i] + 1], core.corners[expected[i] + 2]);
        }
    }
}
