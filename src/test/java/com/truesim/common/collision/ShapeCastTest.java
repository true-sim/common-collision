package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;

import com.truesim.common.math.Quat;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Intersections;
import com.truesim.common.math.random.Rng;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShapeCastTest {

    private static final double EPS = 1e-9;
    /** How exactly casts touch the target. */
    private static final double TOUCH = 1e-9;
    /**
     * How close the two touching points are at contact on a smooth core without a margin (a non-uniformly scaled
     * sphere or capsule). At contact the points come from EPA, whose polytope face is a chord of the curved surface.
     */
    private static final double SMOOTH_CORE_POINTS = 1e-5;

    private final NarrowPhase narrow = new NarrowPhase();
    private final NarrowPhase check = new NarrowPhase();
    private final ShapeCastHit hit = new ShapeCastHit();
    private final Contact contact = new Contact();
    private final ContactBuffer contacts = new ContactBuffer(64);

    private static void assertVec(Vec3 actual, double x, double y, double z, double tolerance) {
        assertThat(actual.x()).isCloseTo(x, within(tolerance));
        assertThat(actual.y()).isCloseTo(y, within(tolerance));
        assertThat(actual.z()).isCloseTo(z, within(tolerance));
    }

    private static Transform moved(Transform pose, Vec3 direction, double t) {
        return pose.withTranslation(pose.translation().add(direction.scale(t)));
    }

    /** Signed distance between the shape moved by t along the direction and the target. */
    private double distanceAt(ConvexShape shape, Transform pose, Vec3 direction, double t, Shape target,
        Transform targetPose) {
        Transform at = moved(pose, direction, t);
        if (target instanceof ConvexShape convex) {
            return check.query(shape, at, convex, targetPose, contact);
        }
        // Within a large distance there is always a nearest triangle near these scenes.
        int count = check.contacts(shape, at, (ConcaveShape) target, targetPose, 10, contacts);
        double nearest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < count; i++) {
            nearest = Math.min(nearest, contacts.get(i).distance());
        }
        return nearest;
    }

    private static boolean isSmoothCore(ConvexShape shape, Transform pose) {
        Vec3 scale = pose.scale();
        return shape.margin() > 0 && !(scale.x() == scale.y() && scale.y() == scale.z());
    }

    /**
     * Checks a cast against sampled distances: on a hit the shape touches the target there and nowhere before, and
     * the hit's points touch and its normal separates; on a miss the shape stays clear of the target all the way.
     */
    private void assertCast(ConvexShape shape, Transform pose, Vec3 direction, double max, Shape target,
        Transform targetPose) {
        boolean found = narrow.cast(shape, pose, direction, max, target, targetPose, hit);
        assertThat(hit.isHit()).isEqualTo(found);
        double end = found ? hit.distance() : max;
        for (int k = 0; k < 20; k++) {
            assertThat(distanceAt(shape, pose, direction, end * k / 20 * (found ? 0.95 : 1), target, targetPose))
                .isPositive();
        }
        if (!found) {
            assertThat(distanceAt(shape, pose, direction, max, target, targetPose)).isPositive();
            return;
        }
        assertThat(hit.distance()).isBetween(0.0, max);
        // Touching the target, up to rounding.
        assertThat(distanceAt(shape, pose, direction, hit.distance(), target, targetPose)).isBetween(-TOUCH, TOUCH);
        boolean smooth = isSmoothCore(shape, pose) || target instanceof ConvexShape convex && isSmoothCore(convex, targetPose);
        assertThat(hit.point().sub(hit.shapePoint()).length()).isLessThanOrEqualTo(smooth ? SMOOTH_CORE_POINTS : TOUCH);
        assertThat(hit.normal().length()).isCloseTo(1, within(EPS));
        // Moving a little along the normal from the impact separates the shapes, unless the normal is a smoothed one.
        if (hit.triangle() == ShapeCastHit.NONE) {
            Transform away = moved(pose, direction, hit.distance()).withTranslation(
                moved(pose, direction, hit.distance()).translation().add(hit.normal().scale(1e-4)));
            assertThat(check.query(shape, away, (ConvexShape) target, targetPose, contact)).isPositive();
        }
    }

    // ---------------------------------------------------------------- convex targets

    @Test
    void sphereOntoSphere() {
        SphereShape ball = new SphereShape(1);
        SphereShape target = new SphereShape(2);
        Transform at = Transform.ofTranslation(Vec3.of(10, 0, 0));
        assertThat(narrow.cast(ball, Transform.IDENTITY, Vec3.of(1, 0, 0), 100, target, at, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(7, within(TOUCH));
        assertThat(hit.isHit()).isTrue();
        assertVec(hit.point(), 8, 0, 0, EPS);
        assertVec(hit.shapePoint(), 8, 0, 0, TOUCH);
        assertVec(hit.normal(), -1, 0, 0, EPS);
        assertThat(hit.pointX()).isCloseTo(8, within(EPS));
        assertThat(hit.pointY()).isCloseTo(0, within(EPS));
        assertThat(hit.pointZ()).isCloseTo(0, within(EPS));
        assertThat(hit.normalX()).isCloseTo(-1, within(EPS));
        assertThat(hit.normalY()).isCloseTo(0, within(EPS));
        assertThat(hit.normalZ()).isCloseTo(0, within(EPS));
        assertThat(hit.triangle()).isEqualTo(ShapeCastHit.NONE);
        assertThat(hit.material()).isEqualTo(ShapeCastHit.NONE);
        assertThat(hit.collider()).isEqualTo(ShapeCastHit.NONE);
        // The direction does not need to be a unit vector: the hit is at a parameter of the motion.
        assertThat(narrow.cast(ball, Transform.IDENTITY, Vec3.of(2, 0, 0), 100, target, at, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(3.5, within(TOUCH));
        // Beyond the maximum, moving away, passing by, not moving.
        assertThat(narrow.cast(ball, Transform.IDENTITY, Vec3.of(1, 0, 0), 6.9, target, at, hit)).isFalse();
        assertThat(hit.isHit()).isFalse();
        assertThat(hit.distance()).isEqualTo(Double.POSITIVE_INFINITY);
        assertThat(narrow.cast(ball, Transform.IDENTITY, Vec3.of(-1, 0, 0), 100, target, at, hit)).isFalse();
        assertThat(narrow.cast(ball, Transform.IDENTITY, Vec3.of(1, 0, 0), 100, target,
            Transform.ofTranslation(Vec3.of(10, 3.5, 0)), hit)).isFalse();
        assertThat(narrow.cast(ball, Transform.IDENTITY, Vec3.ZERO, 100, target, at, hit)).isFalse();
    }

    @Test
    void sweptSpheresMatchInflatedRaycasts() {
        // A sphere of radius r swept against a sphere of radius R is a ray against a sphere of radius R + r.
        Rng rng = Rng.seeded(3);
        int hits = 0;
        for (int i = 0; i < 500; i++) {
            double r = rng.nextDouble(0.1, 1);
            double radius = rng.nextDouble(0.5, 2);
            Vec3 start = rng.nextVec3InSphere().scale(10);
            Vec3 center = rng.nextVec3InSphere().scale(2);
            Vec3 direction = center.add(rng.nextVec3InSphere().scale(3)).sub(start).normalize();
            if (start.sub(center).length() <= r + radius) {
                continue;
            }
            double expected = Intersections.raySphere(start, direction, center, r + radius);
            boolean found = narrow.cast(new SphereShape(r), Transform.ofTranslation(start), direction, 30,
                new SphereShape(radius), Transform.ofTranslation(center), hit);
            assertThat(found).isEqualTo(expected <= 30);
            if (found) {
                hits++;
                assertThat(hit.distance()).isCloseTo(expected, within(TOUCH));
                Vec3 touch = start.add(direction.scale(hit.distance()));
                assertThat(hit.normal().dot(touch.sub(center).normalize())).isCloseTo(1, within(1e-6));
            }
        }
        assertThat(hits).isGreaterThan(50);
    }

    @Test
    void randomConvexCasts() {
        Rng rng = Rng.seeded(7);
        ConvexHullShape hull = ConvexHullShape.of(List.of(Vec3.of(-1, -0.5, 0), Vec3.of(1, -0.5, 0.2),
            Vec3.of(0, 1, -0.3), Vec3.of(0.1, -0.2, 1.2), Vec3.of(-0.4, 0.3, -1)));
        List<ConvexShape> shapes = List.of(new SphereShape(0.7), new CapsuleShape(0.8, 0.3),
            new BoxShape(Vec3.of(0.5, 0.9, 0.4)), hull);
        int hits = 0;
        for (int i = 0; i < 400; i++) {
            ConvexShape shape = shapes.get(i % 4);
            ConvexShape target = shapes.get(i / 4 % 4);
            Transform pose = Transform.of(rng.nextVec3InSphere().scale(6), rng.nextRotation());
            Transform targetPose = Transform.of(rng.nextVec3InSphere(), rng.nextRotation(),
                i % 5 == 0 ? Vec3.of(1.5, 0.7, 1) : Vec3.ONE);
            Vec3 direction = targetPose.translation().add(rng.nextVec3InSphere().scale(1.5)).sub(pose.translation());
            if (check.query(shape, pose, target, targetPose, contact) <= 0) {
                continue;
            }
            assertCast(shape, pose, direction.normalize(), 12, target, targetPose);
            hits += hit.isHit() ? 1 : 0;
        }
        assertThat(hits).isGreaterThan(150);
    }

    /** Lowest y of the box's corners at the pose. */
    private static double lowest(BoxShape box, Transform pose) {
        Vec3 h = box.halfExtents();
        double lowest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 8; i++) {
            lowest = Math.min(lowest, pose.transformPoint(Vec3.of((i & 1) == 0 ? -h.x() : h.x(),
                (i & 2) == 0 ? -h.y() : h.y(), (i & 4) == 0 ? -h.z() : h.z())).y());
        }
        return lowest;
    }

    @Test
    void grazingHitsAreExact() {
        // Boxes gliding down at a shallow angle onto a flat floor, face first or corner first: the exact time of impact
        // and the floor's normal, however shallow the angle. On a large floor GJK's normal is noise when nearly
        // touching; without the secant bound it once let a step overshoot by 0.015 here (into the floor by 1.8e-4).
        BoxShape box = new BoxShape(Vec3.of(0.5, 0.3, 0.7));
        Rng rng = Rng.seeded(13);
        for (double size : new double[] {100, 1e4}) {
            BoxShape floor = new BoxShape(Vec3.of(size, 1, size));
            Transform floorPose = Transform.ofTranslation(Vec3.of(0, -1, 0));
            for (int i = 0; i < 300; i++) {
                Transform pose = Transform.of(Vec3.of(rng.nextDouble(-3, 3), 0, rng.nextDouble(-3, 3)),
                    i % 2 == 0 ? Quat.IDENTITY : rng.nextRotation());
                pose = pose.withTranslation(pose.translation().add(Vec3.of(0, rng.nextDouble(0.5, 1.5) - lowest(box, pose), 0)));
                Vec3 direction = Vec3.of(rng.nextDouble(-1, 1), -rng.nextDouble(0.01, 0.1), rng.nextDouble(-1, 1)).normalize();
                double expected = lowest(box, pose) / -direction.y();
                assertThat(narrow.cast(box, pose, direction, 200, floor, floorPose, hit)).isTrue();
                assertThat(hit.distance()).isCloseTo(expected, within(TOUCH));
                assertThat(hit.normal().dot(Vec3.UP)).isCloseTo(1, within(1e-12));
            }
        }
    }

    @Test
    void startingInContact() {
        BoxShape box = new BoxShape(Vec3.ONE);
        Transform overlapping = Transform.ofTranslation(Vec3.of(1.8, 0.1, 0));
        // Overlapping by 0.2 along x and moving in: hits at once, with the normal that separates them fastest.
        assertThat(narrow.cast(box, Transform.IDENTITY, Vec3.of(1, 0.5, 0), 5, box, overlapping, hit)).isTrue();
        assertThat(hit.distance()).isZero();
        assertVec(hit.normal(), -1, 0, 0, EPS);
        // Sliding along the overlap or moving out of it is free.
        assertThat(narrow.cast(box, Transform.IDENTITY, Vec3.of(0, 1, 0), 5, box, overlapping, hit)).isFalse();
        assertThat(narrow.cast(box, Transform.IDENTITY, Vec3.of(-1, 0, 0), 5, box, overlapping, hit)).isFalse();
        // The same when touching exactly; standing still is no impact either.
        Transform touching = Transform.ofTranslation(Vec3.of(2, 0, 0));
        assertThat(narrow.cast(box, Transform.IDENTITY, Vec3.of(1, 0, 0), 5, box, touching, hit)).isTrue();
        assertThat(hit.distance()).isZero();
        assertVec(hit.normal(), -1, 0, 0, EPS);
        assertThat(narrow.cast(box, Transform.IDENTITY, Vec3.of(0, 0, 1), 5, box, touching, hit)).isFalse();
        assertThat(narrow.cast(box, Transform.IDENTITY, Vec3.of(-1, 0, 0), 5, box, touching, hit)).isFalse();
        assertThat(narrow.cast(box, Transform.IDENTITY, Vec3.ZERO, 5, box, touching, hit)).isFalse();
    }

    @Test
    void slidingAlongTheGround() {
        // A ball resting on the square, cast sideways: the ground does not stop it, a wall does.
        TriangleMeshShape square = square();
        SphereShape ball = new SphereShape(0.25);
        Transform resting = Transform.ofTranslation(Vec3.of(0.5, 0.25, 0.5));
        assertThat(narrow.cast(ball, resting, Vec3.of(1, 0, 1), 1, square, Transform.IDENTITY, hit)).isFalse();
        assertThat(narrow.cast(ball, resting, Vec3.of(1, 0, 0), 1, square, Transform.IDENTITY, hit)).isFalse();
        // Pressing into it is an impact at once.
        assertThat(narrow.cast(ball, resting, Vec3.of(1, -1, 0), 1, square, Transform.IDENTITY, hit)).isTrue();
        assertThat(hit.distance()).isZero();
        assertVec(hit.normal(), 0, 1, 0, EPS);
        CollisionWorld world = new CollisionWorld();
        world.addStatic(square, Transform.IDENTITY);
        int wall = world.addStatic(new BoxShape(Vec3.of(0.1, 1, 1)), Transform.ofTranslation(Vec3.of(1.6, 1, 1)));
        world.update();
        assertThat(world.newQuery().shapeCast(ball, resting, Vec3.of(1, 0, 0), 2, -1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(wall);
        assertThat(hit.distance()).isCloseTo(0.75, within(TOUCH));
    }

    @Test
    void rotatedAndScaledTargets() {
        // A unit sphere scaled into an ellipsoid with x radius 3, rotated a quarter turn about y: its x radius is now along z.
        SphereShape ball = new SphereShape(0.5);
        Transform ellipsoid = Transform.of(Vec3.ZERO, Quat.fromAxisAngle(Vec3.UP, Math.PI / 2), Vec3.of(3, 1, 1));
        assertThat(narrow.cast(ball, Transform.ofTranslation(Vec3.of(0, 0, 10)), Vec3.of(0, 0, -1), 20, new SphereShape(1),
            ellipsoid, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(6.5, within(TOUCH));
        assertVec(hit.normal(), 0, 0, 1, 1e-6);
        assertVec(hit.point(), 0, 0, 3, 1e-6);
    }

    @Test
    void stepLimit() {
        // One step of conservative advancement does not reach the sphere: no hit.
        NarrowPhase limited = new NarrowPhase(1, 128, 256);
        assertThat(limited.cast(new SphereShape(1), Transform.IDENTITY, Vec3.of(1, 0, 0), 100, new BoxShape(Vec3.ONE),
            Transform.of(Vec3.of(10, 0.5, 0), Quat.fromYawPitchRoll(0.4, 0.3, 0.2)), hit)).isFalse();
    }

    @Test
    void castDistanceMustBeFiniteAndNotNegative() {
        SphereShape ball = new SphereShape(1);
        for (double max : new double[] {-1, Double.POSITIVE_INFINITY, Double.NaN}) {
            assertThatIllegalArgumentException()
                .isThrownBy(() -> narrow.cast(ball, Transform.IDENTITY, Vec3.of(1, 0, 0), max, ball, Transform.IDENTITY,
                    hit))
                .withMessageContaining("Cast distance must be finite and not negative");
        }
        assertThat(narrow.cast(ball, Transform.IDENTITY, Vec3.of(1, 0, 0), 0, ball,
            Transform.ofTranslation(Vec3.of(3, 0, 0)), hit)).isFalse();
    }

    // ---------------------------------------------------------------- meshes and heightfields

    /** A flat square from (0, 0, 0) to (2, 0, 2), two triangles with materials 3 and 5, split along x = z. */
    private static TriangleMeshShape square() {
        return TriangleMeshShape.of(List.of(Vec3.ZERO, Vec3.of(2, 0, 0), Vec3.of(2, 0, 2), Vec3.of(0, 0, 2)),
            new int[] {0, 2, 1, 0, 3, 2}, new int[] {3, 5});
    }

    @Test
    void ontoATriangleMesh() {
        TriangleMeshShape square = square();
        BoxShape box = new BoxShape(Vec3.of(0.2, 0.3, 0.2));
        Transform pose = Transform.of(Vec3.of(1.5, 4, 0.5), Quat.IDENTITY);
        assertThat(narrow.cast(box, pose, Vec3.of(0, -1, 0), 10, square, Transform.IDENTITY, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(3.7, within(TOUCH));
        assertVec(hit.normal(), 0, 1, 0, EPS);
        assertThat(hit.pointY()).isCloseTo(0, within(EPS));
        assertThat(hit.triangle()).isZero();
        assertThat(hit.material()).isEqualTo(3);
        assertThat(hit.collider()).isEqualTo(ShapeCastHit.NONE);
        // Over the other triangle.
        assertThat(narrow.cast(box, pose.withTranslation(Vec3.of(0.5, 4, 1.5)), Vec3.of(0, -1, 0), 10, square,
            Transform.IDENTITY, hit)).isTrue();
        assertThat(hit.triangle()).isEqualTo(1);
        assertThat(hit.material()).isEqualTo(5);
        // Straddling the seam both triangles are hit at the same time; the first one found wins.
        assertThat(narrow.cast(box, pose.withTranslation(Vec3.of(1, 4, 1)), Vec3.of(0, -1, 0), 10, square,
            Transform.IDENTITY, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(3.7, within(TOUCH));
        assertVec(hit.normal(), 0, 1, 0, EPS);
        assertThat(hit.triangle()).isIn(0, 1);
        // Beside the square, too short, and from below (triangles are hit from both sides).
        assertThat(narrow.cast(box, pose.withTranslation(Vec3.of(3, 4, 1)), Vec3.of(0, -1, 0), 10, square,
            Transform.IDENTITY, hit)).isFalse();
        assertThat(narrow.cast(box, pose, Vec3.of(0, -1, 0), 3, square, Transform.IDENTITY, hit)).isFalse();
        assertThat(hit.triangle()).isEqualTo(ShapeCastHit.NONE);
        assertThat(narrow.cast(box, pose.withTranslation(Vec3.of(1.5, -2, 0.5)), Vec3.of(0, 1, 0), 10, square,
            Transform.IDENTITY, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(1.7, within(TOUCH));
        assertVec(hit.normal(), 0, -1, 0, EPS);
    }

    @Test
    void nearestTriangleWins() {
        // Two squares, one above the other: a cast down meets the upper one, whichever triangle comes first.
        TriangleMeshShape stack = TriangleMeshShape.of(List.of(Vec3.ZERO, Vec3.of(2, 0, 0), Vec3.of(2, 0, 2),
            Vec3.of(0, 0, 2), Vec3.of(0, 1, 0), Vec3.of(2, 1, 0), Vec3.of(2, 1, 2), Vec3.of(0, 1, 2)),
            new int[] {0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6}, new int[] {0, 0, 1, 1});
        SphereShape ball = new SphereShape(0.25);
        assertThat(narrow.cast(ball, Transform.ofTranslation(Vec3.of(1.5, 5, 0.5)), Vec3.of(0, -1, 0), 10, stack,
            Transform.IDENTITY, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(3.75, within(TOUCH));
        assertThat(hit.material()).isEqualTo(1);
        // From below, the lower one.
        assertThat(narrow.cast(ball, Transform.ofTranslation(Vec3.of(1.5, -5, 0.5)), Vec3.of(0, 1, 0), 10, stack,
            Transform.IDENTITY, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(4.75, within(TOUCH));
        assertThat(hit.material()).isZero();
    }

    @Test
    void placedConcaveShapes() {
        // The square moved up by 1, turned a quarter about y around its corner, and stretched twice along its local x.
        Transform placed = Transform.of(Vec3.of(0, 1, 0), Quat.fromAxisAngle(Vec3.UP, Math.PI / 2), Vec3.of(2, 1, 1));
        Vec3 over = placed.transformPoint(Vec3.of(1.5, 0, 0.5));
        assertThat(narrow.cast(new SphereShape(0.5), Transform.ofTranslation(over.add(Vec3.of(0, 3, 0))),
            Vec3.of(0, -1, 0), 10, square(), placed, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(2.5, within(TOUCH));
        assertVec(hit.point(), over.x(), 1, over.z(), 1e-7);
        assertThat(hit.material()).isEqualTo(3);
    }

    @Test
    void randomCastsOverTerrain() {
        HeightfieldShape field = HeightfieldShapeTest.hills(16, 12);
        TriangleMeshShape mesh = TriangleMeshShapeTest.grid(10);
        Transform fieldPose = Transform.of(Vec3.of(-4, -1, -3), Quat.fromAxisAngle(Vec3.UP, 0.2));
        Rng rng = Rng.seeded(11);
        BoxShape box = new BoxShape(Vec3.of(0.4, 0.3, 0.6));
        SphereShape ball = new SphereShape(0.5);
        int hits = 0;
        for (int i = 0; i < 120; i++) {
            ConvexShape shape = i % 2 == 0 ? ball : box;
            Shape target = i % 3 == 0 ? mesh : field;
            Transform targetPose = i % 3 == 0 ? Transform.IDENTITY : fieldPose;
            Vec3 start = i % 3 == 0 ? Vec3.of(rng.nextDouble(1, 9), 3, rng.nextDouble(1, 9))
                : fieldPose.transformPoint(Vec3.of(rng.nextDouble(4, 28), 4, rng.nextDouble(3, 15)));
            Vec3 direction = Vec3.of(rng.nextDouble(-0.5, 0.5), -1, rng.nextDouble(-0.5, 0.5)).normalize();
            Transform pose = Transform.of(start, rng.nextRotation());
            assertCast(shape, pose, direction, 8, target, targetPose);
            if (hit.isHit()) {
                hits++;
                assertThat(hit.triangle()).isNotEqualTo(ShapeCastHit.NONE);
                assertThat(hit.material()).isEqualTo(((ConcaveShape) target).material(hit.triangle()));
                assertThat(hit.normalY()).isPositive();
            }
        }
        assertThat(hits).isGreaterThan(100);
    }

    // ---------------------------------------------------------------- collision world

    @Test
    void worldCasts() {
        CollisionWorld world = new CollisionWorld();
        int floor = world.addStatic(square(), Transform.ofTranslation(Vec3.of(-1, 0, -1)));
        int baked = world.addStatic(square(), Transform.ofTranslation(Vec3.of(4, 0, -1)), true);
        int pillar = world.addStatic(new BoxShape(Vec3.of(0.5, 2, 0.5)), Transform.ofTranslation(Vec3.of(0, 2, 3)));
        int ball = world.addDynamic(new SphereShape(0.5), Transform.ofTranslation(Vec3.of(0, 1, 0)));
        world.setMaterial(floor, 9);
        world.update();
        CollisionQuery query = world.newQuery();
        SphereShape probe = new SphereShape(0.25);
        // Down from above the ball: the ball, then the floor beneath it once the ball is filtered out.
        Transform above = Transform.ofTranslation(Vec3.of(0, 4, 0));
        assertThat(query.shapeCast(probe, above, Vec3.of(0, -1, 0), 10, -1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(ball);
        assertThat(hit.distance()).isCloseTo(2.25, within(TOUCH));
        assertThat(hit.triangle()).isEqualTo(ShapeCastHit.NONE);
        world.setFilter(ball, 2, -1);
        assertThat(query.shapeCast(probe, above, Vec3.of(0, -1, 0), 10, 1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(floor);
        assertThat(hit.distance()).isCloseTo(3.75, within(TOUCH));
        assertThat(hit.triangle()).isNotEqualTo(ShapeCastHit.NONE);
        assertVec(hit.normal(), 0, 1, 0, EPS);
        // Onto the baked square, whose triangles are in the static tree: its triangle and material.
        assertThat(query.shapeCast(probe, Transform.ofTranslation(Vec3.of(5.5, 4, -0.5)), Vec3.of(0, -1, 0), 10, -1, hit))
            .isTrue();
        assertThat(hit.collider()).isEqualTo(baked);
        assertThat(hit.distance()).isCloseTo(3.75, within(TOUCH));
        assertThat(hit.triangle()).isZero();
        assertThat(hit.material()).isEqualTo(3);
        // Sideways into the pillar; beyond the maximum; nothing there; filtered out.
        Transform beside = Transform.ofTranslation(Vec3.of(-5, 2, 3));
        assertThat(query.shapeCast(probe, beside, Vec3.of(1, 0, 0), 10, -1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(pillar);
        assertThat(hit.distance()).isCloseTo(4.25, within(TOUCH));
        assertVec(hit.normal(), -1, 0, 0, EPS);
        assertThat(query.shapeCast(probe, beside, Vec3.of(1, 0, 0), 4, -1, hit)).isFalse();
        assertThat(hit.isHit()).isFalse();
        assertThat(hit.collider()).isEqualTo(ShapeCastHit.NONE);
        assertThat(query.shapeCast(probe, beside, Vec3.of(-1, 0, 0), 10, -1, hit)).isFalse();
        assertThat(query.shapeCast(probe, Transform.ofTranslation(Vec3.of(5.5, 4, -0.5)), Vec3.of(0, -1, 0), 10, 2, hit))
            .isFalse();
        // Removed colliders are skipped even before the next update.
        world.remove(ball);
        world.remove(baked);
        world.remove(pillar);
        assertThat(query.shapeCast(probe, above, Vec3.of(0, -1, 0), 10, -1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(floor);
        assertThat(hit.material()).isEqualTo(3); // the triangle's material, not the collider's
        assertThat(query.shapeCast(probe, Transform.ofTranslation(Vec3.of(5.5, 4, -0.5)), Vec3.of(0, -1, 0), 10, -1, hit))
            .isFalse();
        assertThat(query.shapeCast(probe, beside, Vec3.of(1, 0, 0), 10, -1, hit)).isFalse();
        assertThatIllegalArgumentException()
            .isThrownBy(() -> query.shapeCast(probe, above, Vec3.of(0, -1, 0), Double.POSITIVE_INFINITY, -1, hit));
    }

    @Test
    void worldCastTiesKeepTheFirstCollider() {
        // Two equal boxes side by side, hit at the same time: the same one wins every time.
        CollisionWorld world = new CollisionWorld();
        world.addDynamic(new BoxShape(Vec3.ONE), Transform.ofTranslation(Vec3.of(-1, 0, 0)));
        world.addDynamic(new BoxShape(Vec3.ONE), Transform.ofTranslation(Vec3.of(1, 0, 0)));
        world.update();
        CollisionQuery query = world.newQuery();
        BoxShape plate = new BoxShape(Vec3.of(2, 0.1, 1));
        assertThat(query.shapeCast(plate, Transform.ofTranslation(Vec3.of(0, 5, 0)), Vec3.of(0, -1, 0), 10, -1, hit))
            .isTrue();
        int first = hit.collider();
        assertThat(hit.distance()).isCloseTo(3.9, within(TOUCH));
        for (int i = 0; i < 3; i++) {
            query.shapeCast(plate, Transform.ofTranslation(Vec3.of(0, 5, 0)), Vec3.of(0, -1, 0), 10, -1, hit);
            assertThat(hit.collider()).isEqualTo(first);
        }
    }

    @Test
    void worldCastsMatchCastsAgainstEachCollider() {
        CollisionWorld world = new CollisionWorld();
        Transform fieldPose = Transform.of(Vec3.of(-4, -1, -3), Quat.fromAxisAngle(Vec3.UP, 0.2));
        world.addStatic(HeightfieldShapeTest.hills(16, 12), fieldPose);
        Rng rng = Rng.seeded(5);
        for (int i = 0; i < 30; i++) {
            ConvexShape shape = i % 2 == 0 ? new SphereShape(rng.nextDouble(0.3, 0.8)) : new BoxShape(Vec3.of(0.5, 0.4, 0.6));
            world.addDynamic(shape, Transform.of(
                Vec3.of(rng.nextDouble(-4, 28), rng.nextDouble(2, 4), rng.nextDouble(-3, 15)), rng.nextRotation()));
        }
        world.update();
        CollisionQuery query = world.newQuery();
        CapsuleShape probe = new CapsuleShape(0.4, 0.3);
        int hits = 0;
        for (int i = 0; i < 200; i++) {
            Transform pose = Transform.of(
                Vec3.of(rng.nextDouble(-4, 28), rng.nextDouble(5, 8), rng.nextDouble(-3, 15)), rng.nextRotation());
            Vec3 direction = Vec3.of(rng.nextDouble(-1, 1), rng.nextDouble(-1, -0.2), rng.nextDouble(-1, 1));
            boolean found = query.shapeCast(probe, pose, direction, 10, -1, hit);
            // The earliest hit among casts against each collider alone.
            double nearest = Double.POSITIVE_INFINITY;
            ShapeCastHit single = new ShapeCastHit();
            for (int c = 0; c < world.size(); c++) {
                if (narrow.cast(probe, pose, direction, 10, world.shape(c), world.pose(c), single)) {
                    nearest = Math.min(nearest, single.distance());
                }
            }
            assertThat(found).isEqualTo(nearest != Double.POSITIVE_INFINITY);
            if (found) {
                hits++;
                assertThat(hit.distance()).isEqualTo(nearest);
                assertThat(narrow.cast(probe, pose, direction, 10, world.shape(hit.collider()),
                    world.pose(hit.collider()), single)).isTrue();
                assertThat(single.distance()).isEqualTo(hit.distance());
            }
        }
        assertThat(hits).isGreaterThan(50);
    }
}
