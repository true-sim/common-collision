package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.truesim.common.jobs.JobSystem;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.random.Rng;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class ParallelCollisionTest {

    private static final int[] THREADS = {1, 2, 3, 8, 100};
    private static final double CONTACT_DISTANCE = 0.05;

    /** A busy scene: baked terrain, a moving mesh platform, and hundreds of moving shapes. */
    private static CollisionWorld scene() {
        CollisionWorld world = new CollisionWorld();
        world.setBoundsMargin(CONTACT_DISTANCE);
        world.addStatic(HeightfieldShapeTest.hills(40, 30), Transform.IDENTITY, true);
        world.addStatic(TriangleMeshShapeTest.grid(12), Transform.ofTranslation(Vec3.of(60, -0.5, 0)));
        world.addDynamic(TriangleMeshShapeTest.grid(4), Transform.ofTranslation(Vec3.of(20, 1, 20)));
        Rng rng = Rng.seeded(41);
        for (int i = 0; i < 300; i++) {
            ConvexShape shape = switch (i % 3) {
                case 0 -> new SphereShape(rng.nextDouble(0.3, 0.9));
                case 1 -> new BoxShape(Vec3.of(rng.nextDouble(0.2, 0.7), rng.nextDouble(0.2, 0.7), rng.nextDouble(0.2, 0.7)));
                default -> new CapsuleShape(rng.nextDouble(0, 0.6), rng.nextDouble(0.2, 0.5));
            };
            world.addDynamic(shape, Transform.of(
                Vec3.of(rng.nextDouble(0, 75), rng.nextDouble(-1.5, 2.5), rng.nextDouble(0, 45)), rng.nextRotation()));
        }
        world.update();
        return world;
    }

    /** Moves every dynamic collider a little, the same way for every run. */
    private static void step(CollisionWorld world, int step) {
        Rng rng = Rng.seeded(100 + step);
        for (int id = 0; id < 400; id++) {
            if (world.contains(id) && !world.isStatic(id)) {
                Transform pose = world.pose(id);
                world.setPose(id, pose.withTranslation(pose.translation().add(rng.nextVec3InSphere().scale(0.3))));
            }
        }
        world.update();
    }

    @Test
    void identicalOnAnyNumberOfThreads() {
        for (int threads : THREADS) {
            CollisionWorld world = scene();
            PairBuffer expectedPairs = new PairBuffer();
            PairBuffer pairs = new PairBuffer(1);
            ContactManifolds expected = new ContactManifolds(4);
            ContactManifolds manifolds = new ContactManifolds(4);
            CollisionQuery reference = world.newQuery();
            try (JobSystem jobs = new JobSystem(threads)) {
                ParallelCollision parallel = new ParallelCollision(world, jobs);
                for (int step = 0; step < 5; step++) {
                    world.findPairs(expectedPairs);
                    expected.prepare(expectedPairs.size());
                    reference.collide(expectedPairs, 0, expectedPairs.size(), CONTACT_DISTANCE, expected);
                    parallel.findPairs(pairs);
                    parallel.collide(pairs, CONTACT_DISTANCE, manifolds);
                    assertSamePairs(pairs, expectedPairs, threads);
                    assertSameContacts(manifolds, expected, threads);
                    step(world, step);
                }
                assertThat(expected.totalContacts()).isGreaterThan(100);
            }
        }
    }

    private static void assertSamePairs(PairBuffer actual, PairBuffer expected, int threads) {
        assertThat(actual.size()).as("%d threads", threads).isEqualTo(expected.size());
        for (int i = 0; i < expected.size(); i++) {
            assertThat(actual.first(i)).isEqualTo(expected.first(i));
            assertThat(actual.second(i)).isEqualTo(expected.second(i));
        }
    }

    private static void assertSameContacts(ContactManifolds actual, ContactManifolds expected, int threads) {
        assertThat(actual.size()).isEqualTo(expected.size());
        for (int i = 0; i < expected.size(); i++) {
            assertThat(actual.count(i)).as("pair %d, %d threads", i, threads).isEqualTo(expected.count(i));
            for (int k = 0; k < expected.count(i); k++) {
                Contact a = actual.get(i, k);
                Contact e = expected.get(i, k);
                // Bit for bit: equal doubles, not just close ones.
                assertThat(a.distance()).isEqualTo(e.distance());
                assertThat(a.normal()).isEqualTo(e.normal());
                assertThat(a.pointA()).isEqualTo(e.pointA());
                assertThat(a.pointB()).isEqualTo(e.pointB());
                assertThat(a.featureA()).isEqualTo(e.featureA());
                assertThat(a.featureB()).isEqualTo(e.featureB());
                assertThat(a.materialB()).isEqualTo(e.materialB());
            }
        }
    }

    @Test
    void growsWithTheWorld() {
        CollisionWorld world = new CollisionWorld();
        world.update();
        try (JobSystem jobs = new JobSystem(3)) {
            ParallelCollision parallel = new ParallelCollision(world, jobs);
            PairBuffer pairs = new PairBuffer();
            ContactManifolds manifolds = new ContactManifolds(2);
            parallel.findPairs(pairs);
            parallel.collide(pairs, 0, manifolds);
            assertThat(pairs.size()).isZero();
            assertThat(manifolds.size()).isZero();
            // A row of touching spheres: every neighbor pair, from more chunks than before.
            List<Integer> ids = new ArrayList<>();
            for (int i = 0; i < 200; i++) {
                ids.add(world.addDynamic(new SphereShape(1), Transform.ofTranslation(Vec3.of(1.9 * i, 0, 0))));
            }
            world.update();
            parallel.findPairs(pairs);
            parallel.collide(pairs, 0, manifolds);
            assertThat(pairs.size()).isEqualTo(199);
            assertThat(manifolds.totalContacts()).isEqualTo(199);
        }
    }

    @Test
    void addAllGrowsTheBuffer() {
        PairBuffer small = new PairBuffer(1);
        PairBuffer many = new PairBuffer();
        for (int i = 0; i < 20; i++) {
            many.add(i, i + 1);
        }
        small.add(-1, -2);
        small.addAll(many);
        assertThat(small.size()).isEqualTo(21);
        assertThat(small.first(0)).isEqualTo(-1);
        assertThat(small.second(20)).isEqualTo(20);
        CollisionQuery query = new CollisionWorld().newQuery();
        assertThatThrownBy(() -> query.findPairs(0, 1, small)).isInstanceOf(IndexOutOfBoundsException.class);
    }
}
