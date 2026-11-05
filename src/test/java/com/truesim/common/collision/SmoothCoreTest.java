package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.truesim.common.math.Quat;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.random.Rng;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Penetration of smooth cores (non-uniformly scaled spheres and capsules) is exact: checked against closed forms, and
 * on random pairs of every kind by certificates (moved out by the depth along the normal, the shapes just touch; no
 * nearby normal needs less; and no other start of the refinement finds less).
 */
class SmoothCoreTest {

    /** Exact to rounding, relative to the shapes' size (about 1). */
    private static final double EXACT = 1e-12;

    private final NarrowPhase narrow = new NarrowPhase();
    private final NarrowPhase check = new NarrowPhase();
    private final Contact contact = new Contact();
    private final Contact touch = new Contact();

    private static Vec3 stretch(Rng rng) {
        return Vec3.of(rng.nextDouble(0.5, 2), rng.nextDouble(0.5, 2), rng.nextDouble(0.5, 2));
    }

    /** How far the cores' difference (plus margins) reaches along n: the depth that separates the shapes along n. */
    private static double reach(ConvexShape a, Transform ta, ConvexShape b, Transform tb, Vec3 n) {
        Pose pa = new Pose();
        Pose pb = new Pose();
        pa.set(a, ta);
        pb.set(b, tb);
        double[] s = new double[3];
        pa.coreSupport(n.x(), n.y(), n.z(), s, 0);
        double h = s[0] * n.x() + s[1] * n.y() + s[2] * n.z();
        pb.coreSupport(-n.x(), -n.y(), -n.z(), s, 0);
        return h - (s[0] * n.x() + s[1] * n.y() + s[2] * n.z()) + pa.margin + pb.margin;
    }

    /** The least depth the refinement finds from 200 directions spread over the sphere. */
    private static double bestOfManyStarts(ConvexShape a, Transform ta, ConvexShape b, Transform tb) {
        Pose pa = new Pose();
        Pose pb = new Pose();
        pa.set(a, ta);
        pb.set(b, tb);
        SmoothPenetration refinement = new SmoothPenetration(pa, pb, 64);
        double best = Double.POSITIVE_INFINITY;
        int starts = 200;
        double[] direction = new double[3];
        for (int i = 0; i < starts; i++) {
            double z = 1 - (2.0 * i + 1) / starts;
            double r = Math.sqrt(1 - z * z);
            direction[0] = r * Math.cos(2.399963229728653 * i);
            direction[1] = r * Math.sin(2.399963229728653 * i);
            direction[2] = z;
            if (refinement.refine(direction[0], direction[1], direction[2], direction, 1)) {
                best = Math.min(best, refinement.normalX * (refinement.ax - refinement.bx)
                    + refinement.normalY * (refinement.ay - refinement.by)
                    + refinement.normalZ * (refinement.az - refinement.bz));
            }
        }
        return best + pa.margin + pb.margin;
    }

    /** Certifies a penetration: touching when moved out, no nearby normal needs less, no other start finds less. */
    private void assertExact(ConvexShape a, Transform ta, ConvexShape b, Transform tb, boolean global) {
        double d = narrow.query(a, ta, b, tb, contact);
        assertThat(d).isNegative();
        Vec3 n = contact.normal();
        assertThat(n.length()).isCloseTo(1, within(EXACT));
        // The depth is the reach along the normal, and the points are that far apart along it.
        assertThat(-d).isCloseTo(reach(a, ta, b, tb, n), within(EXACT));
        Vec3 apart = contact.pointA().sub(contact.pointB());
        assertThat(apart.sub(n.scale(-d)).length()).as("%s %s %s %s", a, ta, b, tb).isLessThan(EXACT);
        // Moved out by the depth along the normal, the shapes just touch.
        Transform out = ta.withTranslation(ta.translation().add(n.scale(d)));
        assertThat(check.query(a, out, b, tb, touch)).as("%s %s %s %s", a, ta, b, tb)
            .isCloseTo(0, within(EXACT));
        // No nearby normal needs less.
        Vec3 t1 = n.cross(Math.abs(n.x()) < 0.6 ? Vec3.of(1, 0, 0) : Vec3.of(0, 1, 0)).normalize();
        Vec3 t2 = n.cross(t1);
        for (double angle : new double[] {1e-6, 1e-3}) {
            for (int k = 0; k < 8; k++) {
                double phi = k * Math.PI / 4;
                Vec3 tilted = n.add(t1.scale(angle * Math.cos(phi))).add(t2.scale(angle * Math.sin(phi))).normalize();
                assertThat(reach(a, ta, b, tb, tilted)).isGreaterThanOrEqualTo(-d - EXACT);
            }
        }
        if (global) {
            assertThat(-d).isLessThanOrEqualTo(bestOfManyStarts(a, ta, b, tb) + EXACT);
        }
    }

    @Test
    void ellipsoidIntoFloor() {
        // The deepest point of an ellipsoid r K (unit ball) + c below y = 0 is r |K^T y| - c.y deep.
        BoxShape floor = new BoxShape(Vec3.of(50, 1, 50));
        Transform floorPose = Transform.ofTranslation(Vec3.of(0, -1, 0));
        SphereShape sphere = new SphereShape(0.7);
        Rng rng = Rng.seeded(4);
        for (int i = 0; i < 200; i++) {
            Vec3 scale = stretch(rng);
            Quat rotation = rng.nextRotation();
            Vec3 axis = rotation.conjugate().rotate(Vec3.UP); // y in the ellipsoid's local frame
            double reach = 0.7 * Vec3.of(axis.x() * scale.x(), axis.y() * scale.y(), axis.z() * scale.z()).length();
            double depth = rng.nextDouble(1e-6, 0.5);
            Transform pose = Transform.of(Vec3.of(rng.nextDouble(-3, 3), reach - depth, rng.nextDouble(-3, 3)),
                rotation, scale);
            assertThat(narrow.query(sphere, pose, floor, floorPose, contact)).isCloseTo(-depth, within(EXACT));
            assertThat(contact.normal().sub(Vec3.of(0, -1, 0)).length()).isLessThan(EXACT);
            assertThat(contact.pointB().y()).isCloseTo(0, within(EXACT));
            assertThat(contact.pointA().y()).isCloseTo(-depth, within(EXACT));
            assertExact(sphere, pose, floor, floorPose, false);
        }
    }

    /** Distance from point p inside the ellipsoid with semi-axes e to its surface: the root of a 1-D equation. */
    private static double insideDistance(Vec3 e, Vec3 p) {
        // The closest surface point is x_i = e_i^2 p_i / (e_i^2 + t), for the t in (-min e_i^2, 0] where |x / e| = 1;
        // the sum below falls from infinity to -1 across that interval: bisection to the last bit.
        double low = -Math.min(e.x() * e.x(), Math.min(e.y() * e.y(), e.z() * e.z()));
        double high = 0;
        for (int i = 0; i < 200; i++) {
            double t = (low + high) / 2;
            if (t == low || t == high) {
                break;
            }
            double fx = e.x() * p.x() / (e.x() * e.x() + t);
            double fy = e.y() * p.y() / (e.y() * e.y() + t);
            double fz = e.z() * p.z() / (e.z() * e.z() + t);
            if (fx * fx + fy * fy + fz * fz > 1) {
                low = t;
            } else {
                high = t;
            }
        }
        double t = (low + high) / 2;
        Vec3 x = Vec3.of(e.x() * e.x() * p.x() / (e.x() * e.x() + t), e.y() * e.y() * p.y() / (e.y() * e.y() + t),
            e.z() * e.z() * p.z() / (e.z() * e.z() + t));
        return x.sub(p).length();
    }

    @Test
    void pointInsideAnEllipsoid() {
        ConvexHullShape point = ConvexHullShape.of(List.of(Vec3.ZERO));
        SphereShape sphere = new SphereShape(0.8);
        Rng rng = Rng.seeded(6);
        int checked = 0;
        while (checked < 200) {
            Vec3 scale = stretch(rng);
            Quat rotation = rng.nextRotation();
            Vec3 center = rng.nextVec3InSphere();
            Vec3 semiAxes = scale.scale(0.8);
            // A point well inside, off the center (where several directions tie).
            Vec3 local = Vec3.of(rng.nextDouble(-0.8, 0.8) * semiAxes.x(), rng.nextDouble(-0.8, 0.8) * semiAxes.y(),
                rng.nextDouble(-0.8, 0.8) * semiAxes.z());
            double inside = local.x() * local.x() / (semiAxes.x() * semiAxes.x())
                + local.y() * local.y() / (semiAxes.y() * semiAxes.y())
                + local.z() * local.z() / (semiAxes.z() * semiAxes.z());
            if (inside > 0.8 || inside < 0.05) {
                continue;
            }
            checked++;
            Transform ellipsoid = Transform.of(center, rotation, scale);
            Transform at = Transform.ofTranslation(center.add(rotation.rotate(local)));
            assertThat(narrow.query(point, at, sphere, ellipsoid, contact))
                .isCloseTo(-insideDistance(semiAxes, local), within(EXACT));
            assertExact(point, at, sphere, ellipsoid, false);
        }
    }

    /** A random pair of the kind that overlaps. */
    private static ConvexShape[] shapes(String kind) {
        ConvexHullShape hull = ConvexHullShape.of(List.of(Vec3.of(1, 0, 0), Vec3.of(-0.5, 0.8, 0),
            Vec3.of(-0.5, -0.8, 0.2), Vec3.of(0, 0.1, 0.9), Vec3.of(0.2, -0.3, -0.7)));
        ConvexHullShape triangle = ConvexHullShape.of(List.of(Vec3.of(1, 0, 0), Vec3.of(-0.5, 0.9, 0),
            Vec3.of(-0.5, -0.9, 0)));
        ConvexHullShape bar = ConvexHullShape.of(List.of(Vec3.of(-1, 0, 0), Vec3.of(1, 0, 0)));
        return switch (kind) {
            case "ellipsoids" -> new ConvexShape[] {new SphereShape(0.6), new SphereShape(0.5)};
            case "ellipsoid, box" -> new ConvexShape[] {new SphereShape(0.6), new BoxShape(Vec3.of(0.8, 0.5, 0.6))};
            case "capsule, box" -> new ConvexShape[] {new CapsuleShape(0.6, 0.3), new BoxShape(Vec3.of(0.8, 0.5, 0.6))};
            case "capsules" -> new ConvexShape[] {new CapsuleShape(0.6, 0.3), new CapsuleShape(0.4, 0.4)};
            case "ellipsoid, hull" -> new ConvexShape[] {new SphereShape(0.6), hull};
            case "capsule, triangle" -> new ConvexShape[] {new CapsuleShape(0.5, 0.4), triangle};
            case "ellipsoid, bar" -> new ConvexShape[] {new SphereShape(0.6), bar};
            default -> new ConvexShape[] {new SphereShape(0.6), new CapsuleShape(0.7, 0.3)}; // uniform capsule
        };
    }

    @Test
    void parallelCapsules() {
        // Parallel segments: the sharp parts' difference is a segment, so EPA's face may name three points on a line.
        CapsuleShape a = new CapsuleShape(0.8, 0.3);
        CapsuleShape b = new CapsuleShape(0.5, 0.4);
        Rng rng = Rng.seeded(12);
        for (int i = 0; i < 100; i++) {
            Vec3 scale = Vec3.of(rng.nextDouble(0.6, 1.8), rng.nextDouble(0.6, 1.8), rng.nextDouble(0.6, 1.8));
            Transform ta = Transform.of(Vec3.ZERO, Quat.IDENTITY, scale);
            Transform tb = Transform.ofTranslation(Vec3.of(rng.nextDouble(-0.5, 0.5), rng.nextDouble(-1.5, 1.5),
                rng.nextDouble(-0.5, 0.5)));
            if (narrow.query(a, ta, b, tb, contact) < 0) {
                assertExact(a, ta, b, tb, true);
                assertExact(b, tb, a, ta, false);
            }
        }
    }

    @Test
    void fewIterationsStillGiveFiniteAnswers() {
        // With too few steps the refinement gives up, and EPA's answer stands.
        SphereShape sphere = new SphereShape(0.6);
        BoxShape box = new BoxShape(Vec3.of(0.8, 0.5, 0.6));
        Transform ta = Transform.of(Vec3.of(0.3, 0.2, 0.1), Quat.fromYawPitchRoll(0.3, 0.2, 0.1),
            Vec3.of(1.7, 0.6, 1.2));
        Transform tb = Transform.of(Vec3.of(0.9, 0.5, 0.4), Quat.fromYawPitchRoll(-0.4, 0.5, 0.9));
        for (int iterations = 1; iterations <= 12; iterations++) {
            double d = new NarrowPhase(iterations, 128, 256).query(sphere, ta, box, tb, contact);
            assertThat(d).isFinite();
        }
    }

    @Test
    void randomPairsAreExact() {
        String[] kinds = {"ellipsoids", "ellipsoid, box", "capsule, box", "capsules", "ellipsoid, hull",
            "capsule, triangle", "ellipsoid, bar", "ellipsoid, capsule"};
        for (String kind : kinds) {
            ConvexShape[] shapes = shapes(kind);
            boolean bothScaled = kind.equals("ellipsoids") || kind.equals("capsules");
            Rng rng = Rng.seeded(kind.hashCode());
            int checked = 0;
            while (checked < 120) {
                Transform ta = Transform.of(rng.nextVec3InSphere().scale(1.5), rng.nextRotation(), stretch(rng));
                Transform tb = Transform.of(rng.nextVec3InSphere().scale(1.5), rng.nextRotation(),
                    bothScaled ? stretch(rng) : Vec3.ONE);
                if (narrow.query(shapes[0], ta, shapes[1], tb, contact) < 0) {
                    assertExact(shapes[0], ta, shapes[1], tb, checked % 3 == 0);
                    // Either order.
                    assertExact(shapes[1], tb, shapes[0], ta, false);
                    checked++;
                }
            }
        }
    }
}
