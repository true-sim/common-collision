package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.truesim.common.math.Quat;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Intersections;
import com.truesim.common.math.random.Rng;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class NarrowPhaseTest {

    private static final double EPS = 1e-9;

    private final NarrowPhase narrow = new NarrowPhase();
    private final Contact contact = new Contact();

    private static Transform at(double x, double y, double z) {
        return Transform.ofTranslation(Vec3.of(x, y, z));
    }

    private static void assertVec(Vec3 actual, double x, double y, double z, double tolerance) {
        assertThat(actual.x()).isCloseTo(x, within(tolerance));
        assertThat(actual.y()).isCloseTo(y, within(tolerance));
        assertThat(actual.z()).isCloseTo(z, within(tolerance));
    }

    // ---------------------------------------------------------------- spheres

    @Test
    void separatedSpheres() {
        SphereShape a = new SphereShape(1);
        SphereShape b = new SphereShape(0.5);
        double d = narrow.query(a, at(0, 0, 0), b, at(3, 4, 0), contact);
        assertThat(d).isCloseTo(3.5, within(EPS));
        assertThat(contact.distance()).isEqualTo(d);
        assertThat(contact.depth()).isZero();
        assertThat(contact.isPenetrating()).isFalse();
        assertVec(contact.normal(), 0.6, 0.8, 0, EPS);
        assertVec(contact.pointA(), 0.6, 0.8, 0, EPS);
        assertVec(contact.pointB(), 3 - 0.3, 4 - 0.4, 0, EPS);
        assertThat(contact.normalX()).isCloseTo(0.6, within(EPS));
        assertThat(contact.normalY()).isCloseTo(0.8, within(EPS));
        assertThat(contact.normalZ()).isCloseTo(0, within(EPS));
        assertThat(narrow.overlaps(a, at(0, 0, 0), b, at(3, 4, 0))).isFalse();
    }

    @Test
    void penetratingSpheresUseTheCoreDistance() {
        SphereShape a = new SphereShape(1);
        double d = narrow.query(a, at(0, 0, 0), a, at(0, 0, 1.5), contact);
        assertThat(d).isCloseTo(-0.5, within(EPS));
        assertThat(contact.depth()).isCloseTo(0.5, within(EPS));
        assertThat(contact.isPenetrating()).isTrue();
        assertVec(contact.normal(), 0, 0, 1, EPS);
        assertVec(contact.pointA(), 0, 0, 1, EPS);
        assertVec(contact.pointB(), 0, 0, 0.5, EPS);
        assertThat(narrow.overlaps(a, at(0, 0, 0), a, at(0, 0, 1.5))).isTrue();
    }

    @Test
    void concentricSpheres() {
        SphereShape a = new SphereShape(1);
        SphereShape b = new SphereShape(2);
        assertThat(narrow.query(a, at(1, 1, 1), b, at(1, 1, 1), contact)).isCloseTo(-3, within(EPS));
        assertVec(contact.normal(), 0, 1, 0, 0);
    }

    @Test
    void scaledSpheres() {
        SphereShape unit = new SphereShape(1);
        // Uniform scale scales the margin.
        Transform big = Transform.of(Vec3.ZERO, Quat.IDENTITY, Vec3.splat(-2));
        assertThat(narrow.query(unit, big, unit, at(5, 0, 0), contact)).isCloseTo(2, within(EPS));
        // Non-uniform scale makes an ellipsoid with semi-axes 3, 1, 1.
        Transform ellipsoid = Transform.of(Vec3.ZERO, Quat.IDENTITY, Vec3.of(3, 1, 1));
        assertThat(narrow.query(unit, ellipsoid, unit, at(5, 0, 0), contact)).isCloseTo(1, within(1e-6));
        assertThat(narrow.query(unit, ellipsoid, unit, at(0, 3, 0), contact)).isCloseTo(1, within(1e-6));
        assertVec(contact.normal(), 0, 1, 0, 1e-6);
    }

    // ---------------------------------------------------------------- capsules

    @Test
    void capsulesMatchSegmentDistance() {
        Rng rng = Rng.seeded(7);
        for (int i = 0; i < 300; i++) {
            CapsuleShape a = new CapsuleShape(rng.nextDouble(0, 2), rng.nextDouble(0.1, 1));
            CapsuleShape b = new CapsuleShape(rng.nextDouble(0, 2), rng.nextDouble(0.1, 1));
            Transform ta = Transform.of(rng.nextVec3InSphere().scale(4), rng.nextRotation());
            Transform tb = Transform.of(rng.nextVec3InSphere().scale(4), rng.nextRotation());
            Vec3 a0 = ta.transformPoint(Vec3.of(0, -a.halfLength(), 0));
            Vec3 a1 = ta.transformPoint(Vec3.of(0, a.halfLength(), 0));
            Vec3 b0 = tb.transformPoint(Vec3.of(0, -b.halfLength(), 0));
            Vec3 b1 = tb.transformPoint(Vec3.of(0, b.halfLength(), 0));
            double core = Math.sqrt(Intersections.segmentSegmentDistanceSquared(a0, a1, b0, b1));
            double d = narrow.query(a, ta, b, tb, contact);
            if (core > 1e-6) {
                assertThat(d).isCloseTo(core - a.radius() - b.radius(), within(1e-7));
                assertThat(contact.pointB().sub(contact.pointA()).length()).isCloseTo(Math.abs(d), within(1e-7));
            } else {
                // Crossing cores: only the radii overlap.
                assertThat(d).isCloseTo(-a.radius() - b.radius(), within(1e-7));
            }
            assertThat(narrow.overlaps(a, ta, b, tb)).isEqualTo(Intersections.capsuleCapsule(a0, a1, a.radius(), b0, b1, b.radius()));
        }
    }

    @Test
    void crossingCapsules() {
        CapsuleShape a = new CapsuleShape(2, 0.5);
        Transform lying = Transform.of(Vec3.of(0, 0, 0), Quat.fromAxisAngle(Vec3.of(0, 0, 1), Math.PI / 2));
        double d = narrow.query(a, at(0, 0, 0), a, lying, contact);
        assertThat(d).isCloseTo(-1, within(EPS));
        assertThat(Math.abs(contact.normalZ())).isCloseTo(1, within(EPS));
    }

    @Test
    void sphereOnCapsuleAxis() {
        CapsuleShape capsule = new CapsuleShape(2, 0.5);
        SphereShape sphere = new SphereShape(0.25);
        // Centers on the core line: any perpendicular normal; Y-axis capsule.
        assertThat(narrow.query(capsule, at(0, 0, 0), sphere, at(0, 1, 0), contact)).isCloseTo(-0.75, within(EPS));
        assertThat(contact.normalY()).isCloseTo(0, within(EPS));
        assertThat(contact.normal().length()).isCloseTo(1, within(EPS));
        // X-axis capsule.
        Transform lying = Transform.ofRotation(Quat.fromAxisAngle(Vec3.of(0, 0, 1), Math.PI / 2));
        assertThat(narrow.query(capsule, lying, sphere, at(1, 0, 0), contact)).isCloseTo(-0.75, within(EPS));
        assertThat(contact.normalX()).isCloseTo(0, within(EPS));
        assertThat(contact.normal().length()).isCloseTo(1, within(EPS));
    }

    @Test
    void flatCoresOffCenter() {
        SphereShape sphere = new SphereShape(0.5);
        // Segment core above its origin, sphere on it: normal from center to center, perpendicular to the segment.
        ConvexHullShape bar = ConvexHullShape.of(List.of(Vec3.of(-1, 1, 0), Vec3.of(1, 1, 0)));
        assertThat(narrow.query(bar, at(0, 0, 0), sphere, at(0.5, 1, 0), contact)).isCloseTo(-0.5, within(EPS));
        assertVec(contact.normal(), 0, 1, 0, EPS);
        // Point core off its origin, sphere on it.
        ConvexHullShape point = ConvexHullShape.of(List.of(Vec3.of(0, 1, 0)));
        assertThat(narrow.query(point, at(0, 0, 0), sphere, at(0, 1, 0), contact)).isCloseTo(-0.5, within(EPS));
        assertVec(contact.normal(), 0, 1, 0, EPS);
        assertThat(point.pointCount()).isEqualTo(1);
        assertThat(point.point(0)).isEqualTo(Vec3.of(0, 1, 0));
    }

    // ---------------------------------------------------------------- boxes

    @Test
    void axisAlignedBoxes() {
        BoxShape a = new BoxShape(Vec3.of(1, 2, 3));
        BoxShape b = new BoxShape(Vec3.of(1, 1, 1));
        assertThat(narrow.query(a, at(0, 0, 0), b, at(3, 0, 0), contact)).isCloseTo(1, within(EPS));
        assertVec(contact.normal(), 1, 0, 0, EPS);
        assertThat(contact.pointA().x()).isCloseTo(1, within(EPS));
        assertThat(contact.pointB().x()).isCloseTo(2, within(EPS));
        // Overlap of 0.25 along y is the shallowest axis.
        assertThat(narrow.query(a, at(0, 0, 0), b, at(1.5, 2.75, 0), contact)).isCloseTo(-0.25, within(EPS));
        assertVec(contact.normal(), 0, 1, 0, EPS);
        assertThat(contact.pointA().y()).isCloseTo(2, within(EPS));
        assertThat(contact.pointB().y()).isCloseTo(1.75, within(EPS));
        // Deep overlap.
        assertThat(narrow.query(a, at(0, 0, 0), b, at(0, 0, 0.5), contact)).isCloseTo(-2, within(EPS));
        assertVec(contact.normal(), 1, 0, 0, EPS);
    }

    @Test
    void sphereAgainstBox() {
        BoxShape box = new BoxShape(Vec3.of(1, 1, 1));
        SphereShape sphere = new SphereShape(0.5);
        // Off the corner.
        assertThat(narrow.query(box, at(0, 0, 0), sphere, at(2, 2, 2), contact))
            .isCloseTo(Math.sqrt(3) - 0.5, within(EPS));
        assertVec(contact.pointA(), 1, 1, 1, EPS);
        // Center inside the box: pushed out through the nearest face.
        assertThat(narrow.query(box, at(0, 0, 0), sphere, at(0.8, 0.1, 0), contact)).isCloseTo(-0.7, within(EPS));
        assertVec(contact.normal(), 1, 0, 0, EPS);
        assertVec(contact.pointA(), 1, 0.1, 0, EPS);
    }

    @Test
    void randomBoxesAreCertified() {
        Rng rng = Rng.seeded(11);
        int separated = 0;
        int penetrating = 0;
        for (int i = 0; i < 2000; i++) {
            Vec3 ha = Vec3.of(rng.nextDouble(0.1, 2), rng.nextDouble(0.1, 2), rng.nextDouble(0.1, 2));
            Vec3 hb = Vec3.of(rng.nextDouble(0.1, 2), rng.nextDouble(0.1, 2), rng.nextDouble(0.1, 2));
            Transform ta = Transform.of(rng.nextVec3InSphere().scale(3), rng.nextRotation());
            Transform tb = Transform.of(rng.nextVec3InSphere().scale(3), rng.nextRotation());
            double d = narrow.query(new BoxShape(ha), ta, new BoxShape(hb), tb, contact);
            Vec3 n = contact.normal();
            assertThat(n.length()).isCloseTo(1, within(1e-9));
            double maxA = support(ha, ta, n);
            double minB = -support(hb, tb, n.negate());
            if (d > 0) {
                separated++;
                // Separating plane certificate: the points are extreme along n and exactly d apart.
                assertThat(maxA).isCloseTo(n.dot(contact.pointA()), within(1e-7));
                assertThat(minB).isCloseTo(n.dot(contact.pointB()), within(1e-7));
                assertThat(contact.pointB().sub(contact.pointA()).length()).isCloseTo(d, within(1e-7));
            } else {
                penetrating++;
                // Moving B by the depth along n separates them, and no SAT axis needs less.
                assertThat(maxA - minB).isCloseTo(-d, within(1e-7));
                assertThat(-d).isCloseTo(minimumOverlap(ha, ta, hb, tb), within(1e-7));
            }
            boolean sat = Intersections.obbObb(ta.translation(), ha, ta.rotation(), tb.translation(), hb, tb.rotation());
            if (Math.abs(d) > 1e-9) {
                assertThat(narrow.overlaps(new BoxShape(ha), ta, new BoxShape(hb), tb)).isEqualTo(sat);
                assertThat(d < 0).isEqualTo(sat);
            }
        }
        assertThat(separated).isGreaterThan(100);
        assertThat(penetrating).isGreaterThan(100);
    }

    @Test
    void hullOfBoxCornersMatchesBox() {
        Vec3 h = Vec3.of(1, 0.5, 2);
        List<Vec3> corners = new ArrayList<>();
        for (int i = 0; i < 8; i++) {
            corners.add(Vec3.of((i & 1) == 0 ? -h.x() : h.x(), (i & 2) == 0 ? -h.y() : h.y(), (i & 4) == 0 ? -h.z() : h.z()));
        }
        corners.add(Vec3.ZERO); // interior points are harmless
        ConvexHullShape hull = ConvexHullShape.of(corners);
        assertThat(hull.localBounds()).isEqualTo(new BoxShape(h).localBounds());
        Rng rng = Rng.seeded(3);
        Contact other = new Contact();
        for (int i = 0; i < 300; i++) {
            Transform ta = Transform.of(rng.nextVec3InSphere().scale(3), rng.nextRotation());
            Transform tb = Transform.of(rng.nextVec3InSphere().scale(3), rng.nextRotation());
            SphereShape sphere = new SphereShape(0.7);
            double expected = narrow.query(new BoxShape(h), ta, sphere, tb, contact);
            double actual = narrow.query(hull, ta, sphere, tb, other);
            assertThat(actual).isCloseTo(expected, within(1e-7));
        }
    }

    @Test
    void nonUniformlyScaledBox() {
        BoxShape unit = new BoxShape(Vec3.ONE);
        Transform stretched = Transform.of(Vec3.ZERO, Quat.IDENTITY, Vec3.of(4, 1, 1));
        assertThat(narrow.query(unit, stretched, unit, at(6, 0, 0), contact)).isCloseTo(1, within(EPS));
        Transform tall = Transform.of(Vec3.ZERO, Quat.IDENTITY, Vec3.of(1, 1, 4));
        assertThat(narrow.query(unit, tall, unit, at(0, 0, 6), contact)).isCloseTo(1, within(EPS));
    }

    @Test
    void fewIterationsStillGiveFiniteAnswers() {
        BoxShape a = new BoxShape(Vec3.of(1, 1, 1));
        Transform ta = Transform.of(Vec3.ZERO, Quat.fromYawPitchRoll(0.3, 0.2, 0.1));
        Transform tb = Transform.of(Vec3.of(1.2, 0.9, 0.7), Quat.fromYawPitchRoll(-0.4, 0.5, 0.9));
        for (int iterations = 1; iterations <= 12; iterations++) {
            assertThat(new NarrowPhase(iterations, 128, 256).query(a, ta, a, tb, contact)).isFinite();
        }
    }

    // ---------------------------------------------------------------- limits and validation

    @Test
    void safetyLimitsStillGiveAnAnswer() {
        BoxShape a = new BoxShape(Vec3.of(1, 1, 1));
        Transform ta = Transform.of(Vec3.ZERO, Quat.fromYawPitchRoll(0.3, 0.2, 0.1));
        Transform tb = Transform.of(Vec3.of(1.2, 0.9, 0.7), Quat.fromYawPitchRoll(-0.4, 0.5, 0.9));
        double exact = narrow.query(a, ta, a, tb, contact);
        assertThat(exact).isNegative();
        // GJK out of iterations reports its best distance so far.
        Transform far = Transform.of(Vec3.of(5, 3, 1), Quat.fromYawPitchRoll(-0.4, 0.5, 0.9));
        assertThat(new NarrowPhase(1, 128, 256).query(a, ta, a, far, contact)).isGreaterThan(0);
        // EPA out of vertices or faces stops with a shallower, still valid estimate.
        assertThat(new NarrowPhase(64, 5, 256).query(a, ta, a, tb, contact)).isBetween(exact, 0.0);
        assertThat(new NarrowPhase(64, 128, 6).query(a, ta, a, tb, contact)).isBetween(exact, 0.0);
    }

    @Test
    void invalidShapesAreRejected() {
        org.assertj.core.api.Assertions.assertThatIllegalArgumentException().isThrownBy(() -> new SphereShape(0));
        org.assertj.core.api.Assertions.assertThatIllegalArgumentException().isThrownBy(() -> new CapsuleShape(-1, 1));
        org.assertj.core.api.Assertions.assertThatIllegalArgumentException().isThrownBy(() -> new CapsuleShape(1, 0));
        org.assertj.core.api.Assertions.assertThatIllegalArgumentException()
            .isThrownBy(() -> new BoxShape(Vec3.of(1, 0, 1)));
        org.assertj.core.api.Assertions.assertThatIllegalArgumentException()
            .isThrownBy(() -> new BoxShape(Vec3.of(0, 1, 1)));
        org.assertj.core.api.Assertions.assertThatIllegalArgumentException()
            .isThrownBy(() -> new BoxShape(Vec3.of(1, 1, 0)));
        org.assertj.core.api.Assertions.assertThatIllegalArgumentException()
            .isThrownBy(() -> ConvexHullShape.of(List.of()));
    }

    @Test
    void localBounds() {
        assertThat(new SphereShape(2).localBounds().max()).isEqualTo(Vec3.of(2, 2, 2));
        assertThat(new CapsuleShape(1, 0.5).localBounds().max()).isEqualTo(Vec3.of(0.5, 1.5, 0.5));
        assertThat(new BoxShape(Vec3.of(1, 2, 3)).localBounds().min()).isEqualTo(Vec3.of(-1, -2, -3));
    }

    // ---------------------------------------------------------------- reference implementations

    /** max over the box's corners of n · corner. */
    private static double support(Vec3 h, Transform t, Vec3 n) {
        double best = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < 8; i++) {
            Vec3 corner = t.transformPoint(
                Vec3.of((i & 1) == 0 ? -h.x() : h.x(), (i & 2) == 0 ? -h.y() : h.y(), (i & 4) == 0 ? -h.z() : h.z()));
            best = Math.max(best, n.dot(corner));
        }
        return best;
    }

    /** Smallest overlap over the 15 separating-axis candidates of two boxes. */
    private static double minimumOverlap(Vec3 ha, Transform ta, Vec3 hb, Transform tb) {
        Vec3[] axesA = {ta.rotation().rotate(Vec3.of(1, 0, 0)), ta.rotation().rotate(Vec3.of(0, 1, 0)),
            ta.rotation().rotate(Vec3.of(0, 0, 1))};
        Vec3[] axesB = {tb.rotation().rotate(Vec3.of(1, 0, 0)), tb.rotation().rotate(Vec3.of(0, 1, 0)),
            tb.rotation().rotate(Vec3.of(0, 0, 1))};
        List<Vec3> axes = new ArrayList<>(List.of(axesA));
        axes.addAll(List.of(axesB));
        for (Vec3 x : axesA) {
            for (Vec3 y : axesB) {
                Vec3 c = x.cross(y);
                if (c.length() > 1e-6) {
                    axes.add(c.normalize());
                }
            }
        }
        double best = Double.POSITIVE_INFINITY;
        for (Vec3 n : axes) {
            double overlapForward = support(ha, ta, n) + support(hb, tb, n.negate());
            double overlapBackward = support(ha, ta, n.negate()) + support(hb, tb, n);
            best = Math.min(best, Math.min(overlapForward, overlapBackward));
        }
        return best;
    }
}
