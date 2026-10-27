package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.truesim.common.math.Quat;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Intersections;
import com.truesim.common.math.geometry.Ray;
import com.truesim.common.math.random.Rng;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class RayCasterTest {

    private static final double EPS = 1e-9;

    private final RayCaster caster = new RayCaster();
    private final RaycastHit hit = new RaycastHit();

    private static void assertVec(Vec3 actual, double x, double y, double z, double tolerance) {
        assertThat(actual.x()).isCloseTo(x, within(tolerance));
        assertThat(actual.y()).isCloseTo(y, within(tolerance));
        assertThat(actual.z()).isCloseTo(z, within(tolerance));
    }

    private boolean cast(Shape shape, Transform transform, Vec3 origin, Vec3 direction, double max) {
        return caster.cast(shape, transform, origin, direction, max, hit);
    }

    /** Asserts the hit's point lies on the ray at its distance and its normal is a unit vector facing the ray. */
    private void assertConsistent(Vec3 origin, Vec3 direction) {
        assertVec(hit.point(), origin.x() + hit.distance() * direction.x(), origin.y() + hit.distance() * direction.y(),
            origin.z() + hit.distance() * direction.z(), 1e-12);
        assertThat(hit.normal().length()).isCloseTo(1, within(EPS));
        assertThat(hit.normal().dot(direction)).isLessThanOrEqualTo(1e-12);
    }

    // ---------------------------------------------------------------- convex shapes

    @Test
    void sphere() {
        SphereShape sphere = new SphereShape(2);
        Transform at = Transform.ofTranslation(Vec3.of(1, 0, 0));
        assertThat(cast(sphere, at, Vec3.of(10, 0, 0), Vec3.of(-1, 0, 0), 100)).isTrue();
        assertThat(hit.distance()).isCloseTo(7, within(EPS));
        assertThat(hit.isHit()).isTrue();
        assertVec(hit.point(), 3, 0, 0, EPS);
        assertVec(hit.normal(), 1, 0, 0, EPS);
        assertThat(hit.pointX()).isCloseTo(3, within(EPS));
        assertThat(hit.pointY()).isZero();
        assertThat(hit.pointZ()).isZero();
        assertThat(hit.normalX()).isCloseTo(1, within(EPS));
        assertThat(hit.normalY()).isZero();
        assertThat(hit.normalZ()).isZero();
        assertThat(hit.triangle()).isEqualTo(RaycastHit.NONE);
        assertThat(hit.material()).isEqualTo(RaycastHit.NONE);
        // Beyond the maximum, pointing away, passing by.
        assertThat(cast(sphere, at, Vec3.of(10, 0, 0), Vec3.of(-1, 0, 0), 6.9)).isFalse();
        assertThat(hit.isHit()).isFalse();
        assertThat(hit.distance()).isEqualTo(Double.POSITIVE_INFINITY);
        assertThat(cast(sphere, at, Vec3.of(10, 0, 0), Vec3.of(1, 0, 0), 100)).isFalse();
        assertThat(cast(sphere, at, Vec3.of(10, 3, 0), Vec3.of(-1, 0, 0), 100)).isFalse();
        // Random rays agree with common-math.
        Rng rng = Rng.seeded(1);
        for (int i = 0; i < 500; i++) {
            Vec3 origin = rng.nextVec3InSphere().scale(10);
            Vec3 direction = rng.nextVec3OnSphere();
            double expected = Intersections.raySphere(origin, direction, Vec3.of(1, 0, 0), 2);
            if (origin.sub(Vec3.of(1, 0, 0)).length() > 2) {
                assertThat(cast(sphere, at, origin, direction, Double.POSITIVE_INFINITY)).isEqualTo(expected != Double.POSITIVE_INFINITY);
                if (hit.isHit()) {
                    assertThat(hit.distance()).isCloseTo(expected, within(1e-9));
                    assertConsistent(origin, direction);
                    assertThat(hit.point().sub(Vec3.of(1, 0, 0)).normalize().dot(hit.normal())).isCloseTo(1, within(EPS));
                }
            }
        }
    }

    @Test
    void startingInsideHitsAtZero() {
        Vec3 direction = Vec3.of(0, 0, 2);
        for (Shape shape : List.of(new SphereShape(1), new CapsuleShape(1, 0.5), new BoxShape(Vec3.ONE),
            ConvexHullShape.of(List.of(Vec3.of(-1, -1, -1), Vec3.of(4, -1, -1), Vec3.of(-1, 4, -1), Vec3.of(-1, -1, 4))))) {
            assertThat(cast(shape, Transform.IDENTITY, Vec3.of(0.1, 0.2, 0.1), direction, 10)).as("%s", shape).isTrue();
            assertThat(hit.distance()).isZero();
            assertVec(hit.normal(), 0, 0, -1, EPS);
            // A negative maximum excludes even that.
            assertThat(cast(shape, Transform.IDENTITY, Vec3.of(0.1, 0.2, 0.1), direction, -1)).isFalse();
        }
    }

    @Test
    void capsule() {
        CapsuleShape capsule = new CapsuleShape(1.5, 0.5);
        Transform pose = Transform.of(Vec3.of(1, 2, 3), Quat.fromYawPitchRoll(0.4, 0.3, 0.2));
        Vec3 a = pose.transformPoint(Vec3.of(0, -1.5, 0));
        Vec3 b = pose.transformPoint(Vec3.of(0, 1.5, 0));
        Rng rng = Rng.seeded(2);
        int hits = 0;
        for (int i = 0; i < 2000; i++) {
            Vec3 origin = Vec3.of(1, 2, 3).add(rng.nextVec3InSphere().scale(5));
            Vec3 direction = rng.nextVec3OnSphere().scale(rng.nextDouble(0.5, 2));
            if (i % 3 == 0) {
                // Aim at the core so many rays hit, some along the axis into the caps.
                Vec3 target = a.interpolate(b, rng.nextDouble(-0.3, 1.3));
                direction = target.sub(origin).scale(0.5);
            }
            double expected = Intersections.rayCapsule(origin, direction, a, b, 0.5);
            boolean found = cast(capsule, pose, origin, direction, Double.POSITIVE_INFINITY);
            assertThat(found).isEqualTo(expected != Double.POSITIVE_INFINITY);
            if (found) {
                hits++;
                assertThat(hit.distance()).isCloseTo(expected, within(1e-9));
                if (hit.distance() > 0) {
                    assertConsistent(origin, direction);
                    double t = Intersections.segmentClosestParameter(a, b, hit.point());
                    Vec3 axis = a.interpolate(b, t);
                    assertThat(hit.point().sub(axis).length()).isCloseTo(0.5, within(1e-9));
                    assertThat(hit.point().sub(axis).normalize().dot(hit.normal())).isCloseTo(1, within(1e-9));
                }
            }
        }
        assertThat(hits).isGreaterThan(500);
        // Straight down the axis onto the top cap, and parallel to the axis past the side.
        assertThat(cast(capsule, Transform.IDENTITY, Vec3.of(0, 5, 0), Vec3.of(0, -1, 0), 10)).isTrue();
        assertThat(hit.distance()).isCloseTo(3, within(EPS));
        assertVec(hit.normal(), 0, 1, 0, EPS);
        assertThat(cast(capsule, Transform.IDENTITY, Vec3.of(0.6, 5, 0), Vec3.of(0, -1, 0), 10)).isFalse();
        // Sideways onto the body.
        assertThat(cast(capsule, Transform.IDENTITY, Vec3.of(3, 0.5, 0), Vec3.of(-1, 0, 0), 10)).isTrue();
        assertThat(hit.distance()).isCloseTo(2.5, within(EPS));
        assertVec(hit.normal(), 1, 0, 0, EPS);
    }

    @Test
    void box() {
        BoxShape box = new BoxShape(Vec3.of(1, 2, 3));
        Quat rotation = Quat.fromYawPitchRoll(0.7, -0.2, 0.5);
        Transform pose = Transform.of(Vec3.of(-1, 0, 2), rotation);
        Rng rng = Rng.seeded(3);
        int hits = 0;
        for (int i = 0; i < 2000; i++) {
            Vec3 origin = Vec3.of(-1, 0, 2).add(rng.nextVec3InSphere().scale(8));
            Vec3 direction = rng.nextVec3OnSphere();
            if (i % 2 == 0) {
                direction = Vec3.of(-1, 0, 2).add(rng.nextVec3InSphere().scale(2)).sub(origin);
            }
            double expected = Intersections.rayObb(origin, direction, Vec3.of(-1, 0, 2), Vec3.of(1, 2, 3), rotation);
            boolean found = cast(box, pose, origin, direction, Double.POSITIVE_INFINITY);
            assertThat(found).isEqualTo(expected != Double.POSITIVE_INFINITY);
            if (found && hit.distance() > 0) {
                hits++;
                assertThat(hit.distance()).isCloseTo(expected, within(1e-9));
                assertConsistent(origin, direction);
                // The normal is one of the box's face normals.
                Vec3 local = rotation.conjugate().rotate(hit.normal());
                assertThat(Math.max(Math.abs(local.x()), Math.max(Math.abs(local.y()), Math.abs(local.z()))))
                    .isCloseTo(1, within(1e-9));
            }
        }
        assertThat(hits).isGreaterThan(500);
        // Axis-parallel rays: outside a slab misses, inside it hits.
        assertThat(cast(box, Transform.IDENTITY, Vec3.of(5, 0, 0), Vec3.of(-1, 0, 0), 10)).isTrue();
        assertVec(hit.normal(), 1, 0, 0, EPS);
        assertThat(cast(box, Transform.IDENTITY, Vec3.of(-5, 0, 0), Vec3.of(1, 0, 0), 10)).isTrue();
        assertVec(hit.normal(), -1, 0, 0, EPS);
        assertThat(cast(box, Transform.IDENTITY, Vec3.of(5, 2.5, 0), Vec3.of(-1, 0, 0), 10)).isFalse();
        assertThat(cast(box, Transform.IDENTITY, Vec3.of(0, 0, 5), Vec3.of(0, 0, -1), 10)).isTrue();
        assertVec(hit.normal(), 0, 0, 1, EPS);
        assertThat(cast(box, Transform.IDENTITY, Vec3.of(5, 0, 0), Vec3.of(1, 0, 0), 10)).isFalse();
    }

    @Test
    void hullMatchesBox() {
        Vec3 h = Vec3.of(1, 0.5, 2);
        List<Vec3> corners = new ArrayList<>();
        for (int i = 0; i < 8; i++) {
            corners.add(Vec3.of((i & 1) == 0 ? -h.x() : h.x(), (i & 2) == 0 ? -h.y() : h.y(), (i & 4) == 0 ? -h.z() : h.z()));
        }
        ConvexHullShape hull = ConvexHullShape.of(corners);
        BoxShape box = new BoxShape(h);
        Transform pose = Transform.of(Vec3.of(0.5, 1, -1), Quat.fromYawPitchRoll(-0.3, 0.6, 0.1));
        RaycastHit boxHit = new RaycastHit();
        Rng rng = Rng.seeded(4);
        int hits = 0;
        for (int i = 0; i < 1000; i++) {
            Vec3 origin = rng.nextVec3InSphere().scale(6);
            Vec3 direction = Vec3.of(0.5, 1, -1).add(rng.nextVec3InSphere().scale(2.5)).sub(origin);
            boolean expected = caster.cast(box, pose, origin, direction, 2, boxHit);
            boolean found = cast(hull, pose, origin, direction, 2);
            // Rays grazing an edge or ending right at the surface may legitimately differ.
            if (Math.abs(boxHit.distance() - 2) > 1e-6) {
                assertThat(found).isEqualTo(expected);
            }
            if (found && expected) {
                hits++;
                assertThat(hit.distance()).isCloseTo(boxHit.distance(), within(1e-7));
                assertVec(hit.point(), boxHit.pointX(), boxHit.pointY(), boxHit.pointZ(), 1e-6);
                if (hit.distance() > 0) {
                    assertThat(hit.normal().dot(direction)).isNegative();
                }
            }
        }
        assertThat(hits).isGreaterThan(200);
        // Moving away from the hull, and stopping short of it.
        assertThat(cast(hull, Transform.IDENTITY, Vec3.of(5, 0, 0), Vec3.of(1, 0, 0), 10)).isFalse();
        assertThat(cast(hull, Transform.IDENTITY, Vec3.of(5, 0, 0), Vec3.of(-1, 0, 0), 3)).isFalse();
        assertThat(cast(hull, Transform.IDENTITY, Vec3.of(5, 0, 0), Vec3.of(-1, 0, 0), 4)).isTrue();
        assertThat(hit.distance()).isCloseTo(4, within(EPS));
        assertVec(hit.normal(), 1, 0, 0, EPS);
    }

    @Test
    void hullGivesUpAfterTooManySteps() {
        // Approaching the slanted face takes several steps: past the limit the ray counts as a miss.
        ConvexHullShape tetrahedron = ConvexHullShape.of(List.of(Vec3.ZERO, Vec3.UNIT_X, Vec3.UNIT_Y, Vec3.UNIT_Z));
        assertThat(new RayCaster(1).cast(tetrahedron, Transform.IDENTITY, Vec3.of(0.2, 0.2, 5), Vec3.of(0, 0, -1), 10, hit))
            .isFalse();
        assertThat(caster.cast(tetrahedron, Transform.IDENTITY, Vec3.of(0.2, 0.2, 5), Vec3.of(0, 0, -1), 10, hit))
            .isTrue();
        assertThat(hit.distance()).isCloseTo(4.4, within(EPS));
    }

    @Test
    void scaledShapes() {
        SphereShape unit = new SphereShape(1);
        Transform ellipsoid = Transform.of(Vec3.ZERO, Quat.IDENTITY, Vec3.of(3, 1, 1));
        assertThat(cast(unit, ellipsoid, Vec3.of(10, 0, 0), Vec3.of(-1, 0, 0), 100)).isTrue();
        assertThat(hit.distance()).isCloseTo(7, within(EPS));
        assertVec(hit.normal(), 1, 0, 0, EPS);
        assertThat(cast(unit, ellipsoid, Vec3.of(0, 5, 0), Vec3.of(0, -1, 0), 100)).isTrue();
        assertThat(hit.distance()).isCloseTo(4, within(EPS));
        // Oblique: the point is on the ellipsoid and the normal is its gradient.
        Vec3 origin = Vec3.of(4, 3, 1);
        Vec3 direction = Vec3.of(-1, -1, -0.2);
        assertThat(cast(unit, ellipsoid, origin, direction, 100)).isTrue();
        Vec3 p = hit.point();
        assertThat(p.x() * p.x() / 9 + p.y() * p.y() + p.z() * p.z()).isCloseTo(1, within(1e-9));
        assertThat(Vec3.of(p.x() / 9, p.y(), p.z()).normalize().dot(hit.normal())).isCloseTo(1, within(1e-9));
        assertConsistent(origin, direction);
    }

    @Test
    void overloads() {
        SphereShape sphere = new SphereShape(1);
        assertThat(caster.cast(sphere, Transform.IDENTITY, new Ray(Vec3.of(0, 0, 5), Vec3.of(0, 0, -1)), 10, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(4, within(EPS));
        assertThat(caster.cast(sphere, Transform.IDENTITY, 0, 5, 0, 0, -1, 0, 10, hit)).isTrue();
        assertVec(hit.normal(), 0, 1, 0, EPS);
    }

    // ---------------------------------------------------------------- triangles

    @Test
    void meshMatchesBruteForce() {
        TriangleMeshShape mesh = TriangleMeshShapeTest.grid(12);
        Transform pose = Transform.of(Vec3.of(-6, 1, 2), Quat.fromYawPitchRoll(0.3, 0.1, -0.2), Vec3.of(1, 2, 1));
        Rng rng = Rng.seeded(6);
        int hits = 0;
        for (int i = 0; i < 1000; i++) {
            Vec3 origin = pose.transformPoint(Vec3.of(rng.nextDouble(-2, 14), rng.nextDouble(-3, 3), rng.nextDouble(-2, 14)));
            Vec3 direction = rng.nextVec3OnSphere().scale(rng.nextDouble(0.5, 2));
            assertMatchesBruteForce(mesh, pose, origin, direction, 20);
            if (hit.isHit()) {
                hits++;
            }
        }
        assertThat(hits).isGreaterThan(150);
    }

    @Test
    void heightfieldMatchesBruteForce() {
        HeightfieldShape field = HeightfieldShapeTest.hills(24, 18);
        Transform pose = Transform.of(Vec3.of(3, -1, 4), Quat.fromYawPitchRoll(0.5, 0, 0));
        Rng rng = Rng.seeded(8);
        int hits = 0;
        for (int i = 0; i < 3000; i++) {
            Vec3 local = Vec3.of(rng.nextDouble(-6, 54), rng.nextDouble(-4, 5), rng.nextDouble(-6, 33));
            Vec3 direction = switch (i % 5) {
                case 0 -> Vec3.of(0, -1, 0); // straight down
                case 1 -> Vec3.of(rng.nextDouble(-1, 1), 0, rng.nextDouble(-1, 1)); // level
                case 2 -> Vec3.of(rng.nextDouble(-1, 1), 0, 0); // along x
                default -> rng.nextVec3OnSphere();
            };
            Vec3 origin = pose.transformPoint(local);
            assertMatchesBruteForce(field, pose, origin, pose.transformDirection(direction), i % 7 == 0 ? 3 : 100);
            if (hit.isHit()) {
                hits++;
            }
        }
        assertThat(hits).isGreaterThan(400);
        // Straight down onto a known point in cell (1, 1), and from below.
        HeightfieldShape flat = HeightfieldShape.of(3, 3, 1, 1, new double[9], new int[] {1, 2, 3, 4});
        assertThat(cast(flat, Transform.IDENTITY, Vec3.of(1.25, 2, 1.5), Vec3.of(0, -1, 0), 10)).isTrue();
        assertThat(hit.distance()).isCloseTo(2, within(EPS));
        assertVec(hit.normal(), 0, 1, 0, EPS);
        assertThat(hit.material()).isEqualTo(4);
        assertThat(hit.triangle() / 2).isEqualTo(3);
        assertThat(cast(flat, Transform.IDENTITY, Vec3.of(1.25, -2, 1.5), Vec3.of(0, 1, 0), 10)).isTrue();
        assertVec(hit.normal(), 0, -1, 0, EPS);
        // Level ray in the plane of a flat field, and rays missing the bounds.
        assertThat(cast(flat, Transform.IDENTITY, Vec3.of(-1, 1, 1), Vec3.of(1, 0, 0), 10)).isFalse();
        assertThat(cast(flat, Transform.IDENTITY, Vec3.of(-1, 0.5, 1), Vec3.of(-1, -1, 0), 10)).isFalse();
        assertThat(cast(flat, Transform.IDENTITY, Vec3.of(1, 5, 1), Vec3.of(0, -1, 0), 2)).isFalse();
    }

    private void assertMatchesBruteForce(ConcaveShape shape, Transform pose, Vec3 origin, Vec3 direction, double max) {
        double best = Double.POSITIVE_INFINITY;
        int bestTriangle = -1;
        for (int t = 0; t < shape.triangleCount(); t++) {
            double[] c = new double[9];
            shape.triangle(t, c, 0);
            Vec3 a = pose.transformPoint(Vec3.of(c[0], c[1], c[2]));
            Vec3 b = pose.transformPoint(Vec3.of(c[3], c[4], c[5]));
            Vec3 d = pose.transformPoint(Vec3.of(c[6], c[7], c[8]));
            double distance = Intersections.rayTriangle(origin, direction, a, b, d);
            if (distance < best) {
                best = distance;
                bestTriangle = t;
            }
        }
        boolean expected = best <= max;
        boolean found = cast(shape, pose, origin, direction, max);
        // Rays ending right at a triangle may legitimately differ.
        if (Math.abs(best - max) > 1e-9) {
            assertThat(found).as("ray %s %s", origin, direction).isEqualTo(expected);
        }
        if (found && expected) {
            assertThat(hit.distance()).isCloseTo(best, within(1e-9));
            assertConsistent(origin, direction);
            // On a shared edge either triangle may be reported; both are at the same distance.
            if (hit.triangle() != bestTriangle) {
                double[] c = new double[9];
                shape.triangle(hit.triangle(), c, 0);
                Vec3 a = pose.transformPoint(Vec3.of(c[0], c[1], c[2]));
                Vec3 b = pose.transformPoint(Vec3.of(c[3], c[4], c[5]));
                Vec3 d = pose.transformPoint(Vec3.of(c[6], c[7], c[8]));
                assertThat(Intersections.rayTriangle(origin, direction, a, b, d)).isCloseTo(best, within(1e-9));
            }
            assertThat(hit.material()).isEqualTo(shape.material(hit.triangle()));
        }
    }
}
