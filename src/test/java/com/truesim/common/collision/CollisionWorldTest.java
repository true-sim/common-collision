package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatIllegalStateException;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.truesim.common.math.Quat;
import com.truesim.common.math.Transform;
import com.truesim.common.math.TransformArray;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;
import com.truesim.common.math.random.Rng;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import org.junit.jupiter.api.Test;

class CollisionWorldTest {

    private final CollisionWorld world = new CollisionWorld();
    private final RayCaster caster = new RayCaster();
    private final NarrowPhase narrow = new NarrowPhase();
    private final List<Integer> dynamics = new ArrayList<>();

    /** A floor mesh, a heightfield, some static boxes and many moving shapes. */
    private void populate(Rng rng, int moving) {
        world.addStatic(TriangleContactsTest.meshOf(HeightfieldShape.of(9, 9, 2, 2, new double[81])),
            Transform.ofTranslation(Vec3.of(-8, 0, -8)));
        world.addStatic(HeightfieldShapeTest.hills(10, 10), Transform.ofTranslation(Vec3.of(10, -1, -10)));
        for (int i = 0; i < 5; i++) {
            world.addStatic(new BoxShape(Vec3.of(1, 2, 1)), Transform.of(randomPoint(rng), rng.nextRotation()));
        }
        for (int i = 0; i < moving; i++) {
            dynamics.add(world.addDynamic(randomShape(rng, i), randomPose(rng)));
        }
    }

    private static Vec3 randomPoint(Rng rng) {
        return Vec3.of(rng.nextDouble(-10, 30), rng.nextDouble(-1, 4), rng.nextDouble(-10, 10));
    }

    private static Transform randomPose(Rng rng) {
        return Transform.of(randomPoint(rng), rng.nextRotation(), Vec3.splat(rng.nextDouble(0.6, 1.4)));
    }

    private static ConvexShape randomShape(Rng rng, int i) {
        return switch (i % 4) {
            case 0 -> new SphereShape(rng.nextDouble(0.3, 1.2));
            case 1 -> new BoxShape(Vec3.of(rng.nextDouble(0.2, 1), rng.nextDouble(0.2, 1), rng.nextDouble(0.2, 1)));
            case 2 -> new CapsuleShape(rng.nextDouble(0, 1), rng.nextDouble(0.2, 0.6));
            default -> ConvexHullShape.of(List.of(Vec3.ZERO, Vec3.of(1, 0, 0), Vec3.of(0, 1, 0), Vec3.of(0, 0, 1)));
        };
    }

    private Set<String> pairsOf(PairBuffer pairs) {
        Set<String> set = new TreeSet<>();
        for (int i = 0; i < pairs.size(); i++) {
            int a = pairs.first(i);
            int b = pairs.second(i);
            assertThat(world.isStatic(a)).isFalse();
            set.add(Math.min(a, b) + "-" + Math.max(a, b));
        }
        assertThat(set).hasSize(pairs.size()); // no duplicates
        return set;
    }

    private Set<String> bruteForcePairs() {
        Set<String> set = new TreeSet<>();
        List<Integer> ids = new ArrayList<>();
        for (int slot = 0; slot < 200; slot++) {
            for (int generation = 0; generation < 4; generation++) { // ids carry their slot's generation
                int id = slot | generation << 24;
                if (world.contains(id)) {
                    ids.add(id);
                }
            }
        }
        for (int a : ids) {
            for (int b : ids) {
                boolean concave = world.shape(a) instanceof ConcaveShape && world.shape(b) instanceof ConcaveShape;
                if (a < b && !(world.isStatic(a) && world.isStatic(b)) && !concave
                    && (world.group(a) & world.mask(b)) != 0 && (world.group(b) & world.mask(a)) != 0
                    && world.bounds(a).intersects(world.bounds(b))) {
                    set.add(a + "-" + b);
                }
            }
        }
        return set;
    }

    // ---------------------------------------------------------------- broad phase

    @Test
    void pairsMatchBruteForce() {
        Rng rng = Rng.seeded(1);
        populate(rng, 60);
        PairBuffer pairs = new PairBuffer(1);
        for (int step = 0; step < 10; step++) {
            world.update();
            world.findPairs(pairs);
            assertThat(pairsOf(pairs)).isEqualTo(bruteForcePairs());
            for (int id : dynamics) {
                Transform pose = world.pose(id);
                world.setPose(id, pose.withTranslation(pose.translation().add(rng.nextVec3InSphere())));
            }
        }
        assertThat(pairs.size()).isGreaterThan(10);
    }

    @Test
    void filtersAndStructuralChanges() {
        Rng rng = Rng.seeded(2);
        populate(rng, 30);
        // Group 2 only collides with things whose mask has bit 2, and its mask excludes group 1.
        for (int i = 0; i < 10; i++) {
            world.setFilter(dynamics.get(i), 2, 2);
        }
        for (int i = 0; i < 5; i++) {
            world.remove(dynamics.get(20 + i));
        }
        int reused = world.addDynamic(new SphereShape(3), Transform.ofTranslation(Vec3.of(0, 1, 0)));
        // The last freed slot comes back first, under a new id: the old one names nothing.
        assertThat(reused & 0xffffff).isEqualTo(dynamics.get(24));
        assertThat(reused).isNotEqualTo(dynamics.get(24));
        assertThat(world.contains(dynamics.get(24))).isFalse();
        int stale = dynamics.get(24);
        assertThatIllegalArgumentException().isThrownBy(() -> world.shape(stale))
            .withMessageEndingWith("(removed: its slot holds a newer one)");
        world.update();
        PairBuffer pairs = new PairBuffer();
        world.findPairs(pairs);
        assertThat(pairsOf(pairs)).isEqualTo(bruteForcePairs());
        assertThat(world.size()).isEqualTo(7 + 30 - 5 + 1);
        // A collider removed after the update is skipped until the next one.
        world.remove(reused);
        world.findPairs(pairs);
        assertThat(pairsOf(pairs)).isEqualTo(bruteForcePairs());
        assertThat(world.group(dynamics.get(0))).isEqualTo(2);
        assertThat(world.mask(dynamics.get(0))).isEqualTo(2);
    }

    @Test
    void emptyWorld() {
        world.update();
        PairBuffer pairs = new PairBuffer();
        world.findPairs(pairs);
        assertThat(pairs.size()).isZero();
        CollisionQuery query = world.newQuery();
        RaycastHit hit = new RaycastHit();
        assertThat(query.raycast(Vec3.ZERO, Vec3.UNIT_X, 100, -1, hit)).isFalse();
        assertThat(hit.collider()).isEqualTo(RaycastHit.NONE);
        assertThat(query.overlap(new SphereShape(1), Transform.IDENTITY, -1, collider -> true)).isZero();
    }

    // ---------------------------------------------------------------- queries

    @Test
    void raycastsMatchBruteForce() {
        Rng rng = Rng.seeded(3);
        populate(rng, 50);
        world.setFilter(dynamics.get(0), 4, -1);
        world.update();
        CollisionQuery query = world.newQuery();
        RaycastHit hit = new RaycastHit();
        RaycastHit single = new RaycastHit();
        int hits = 0;
        for (int i = 0; i < 500; i++) {
            Vec3 origin = randomPoint(rng).add(Vec3.of(0, 3, 0));
            Vec3 direction = rng.nextVec3OnSphere();
            int mask = i % 5 == 0 ? 4 : ~4;
            double best = Double.POSITIVE_INFINITY;
            int bestCollider = RaycastHit.NONE;
            for (int id = 0; id < 100; id++) {
                if (world.contains(id) && (world.group(id) & mask) != 0
                    && caster.cast(world.shape(id), world.pose(id), origin, direction, 30, single)
                    && single.distance() < best) {
                    best = single.distance();
                    bestCollider = id;
                }
            }
            boolean found = query.raycast(origin, direction, 30, mask, hit);
            assertThat(found).isEqualTo(bestCollider != RaycastHit.NONE);
            if (found) {
                hits++;
                assertThat(hit.distance()).isCloseTo(best, within(1e-9));
                assertThat(hit.collider()).isEqualTo(bestCollider);
                assertThat(world.contains(hit.collider())).isTrue();
            } else {
                assertThat(hit.collider()).isEqualTo(RaycastHit.NONE);
                assertThat(hit.isHit()).isFalse();
            }
        }
        assertThat(hits).isGreaterThan(100);
        // Straight down onto the floor mesh, raw-double overload.
        assertThat(query.raycast(-4, 10, -4, 0, -1, 0, 100, -1, hit)).isTrue();
        assertThat(hit.collider()).isZero();
        assertThat(hit.distance()).isCloseTo(10, within(1e-9));
        // A direct RayCaster cast does not name a collider.
        caster.cast(world.shape(0), world.pose(0), Vec3.of(-4, 10, -4), Vec3.of(0, -1, 0), 100, hit);
        assertThat(hit.collider()).isEqualTo(RaycastHit.NONE);
    }

    @Test
    void overlapsMatchBruteForce() {
        Rng rng = Rng.seeded(4);
        populate(rng, 50);
        world.update();
        CollisionQuery query = world.newQuery();
        Contact contact = new Contact();
        ContactBuffer buffer = new ContactBuffer(4);
        for (int i = 0; i < 200; i++) {
            ConvexShape shape = randomShape(rng, i);
            Transform pose = randomPose(rng);
            int mask = i % 3 == 0 ? 1 : -1;
            Set<Integer> expected = new TreeSet<>();
            for (int id = 0; id < 100; id++) {
                if (!world.contains(id) || (world.group(id) & mask) == 0) {
                    continue;
                }
                boolean touching = world.shape(id) instanceof ConvexShape convex
                    ? narrow.query(shape, pose, convex, world.pose(id), contact) <= 0
                    : narrow.contacts(shape, pose, (ConcaveShape) world.shape(id), world.pose(id), 0, buffer) > 0;
                if (touching) {
                    expected.add(id);
                }
            }
            Set<Integer> found = new TreeSet<>();
            int count = query.overlap(shape, pose, mask, collider -> found.add(collider));
            assertThat(found).isEqualTo(expected);
            assertThat(count).isEqualTo(expected.size());
        }
        // Stopping early: in the static tree, and in the dynamic tree.
        Transform onFloor = Transform.ofTranslation(Vec3.of(-4, 0, -4));
        assertThat(query.overlap(new SphereShape(1), onFloor, -1, collider -> false)).isEqualTo(1);
        int moving = world.addDynamic(new SphereShape(1), Transform.ofTranslation(Vec3.of(50, 50, 50)));
        world.addDynamic(new SphereShape(1), Transform.ofTranslation(Vec3.of(50.5, 50, 50)));
        world.update();
        assertThat(query.overlap(new SphereShape(1), world.pose(moving), -1, collider -> false)).isEqualTo(1);
    }

    // ---------------------------------------------------------------- poses

    @Test
    void movingCollidersAndTrees() {
        int box = world.addStatic(new BoxShape(Vec3.ONE), Transform.IDENTITY);
        int ball = world.addDynamic(new SphereShape(1), Transform.ofTranslation(Vec3.of(10, 0, 0)));
        world.update();
        CollisionQuery query = world.newQuery();
        RaycastHit hit = new RaycastHit();
        assertThat(query.raycast(Vec3.of(10, 10, 0), Vec3.of(0, -1, 0), 100, -1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(ball);
        // Move the static box under the ray: the static tree is rebuilt.
        world.setPose(box, Transform.ofTranslation(Vec3.of(20, 0, 0)));
        world.update();
        assertThat(query.raycast(Vec3.of(20, 10, 0), Vec3.of(0, -1, 0), 100, -1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(box);
        // Move the ball away: with a threshold of 0 every refit is followed by a rebuild.
        world.setRebuildThreshold(0);
        assertThat(world.rebuildThreshold()).isZero();
        for (int i = 0; i < 20; i++) {
            world.addDynamic(new SphereShape(0.5), Transform.ofTranslation(Vec3.of(i, 5, 0)));
        }
        world.update();
        world.setPose(ball, Transform.ofTranslation(Vec3.of(-100, 0, 0)));
        world.update();
        assertThat(world.dynamicTree.degradation()).isCloseTo(1, within(1e-12));
        assertThat(query.raycast(Vec3.of(-100, 10, 0), Vec3.of(0, -1, 0), 100, -1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(ball);
        // World bounds follow rotation and scale exactly.
        world.setPose(box, Transform.of(Vec3.of(1, 2, 3), Quat.fromAxisAngle(Vec3.UNIT_Z, Math.PI / 4), Vec3.of(2, 1, 1)));
        Aabb bounds = world.bounds(box);
        double reach = (2 + 1) * Math.sqrt(0.5);
        assertThat(bounds.min().x()).isCloseTo(1 - reach, within(1e-12));
        assertThat(bounds.max().y()).isCloseTo(2 + reach, within(1e-12));
        assertThat(bounds.max().z()).isCloseTo(4, within(1e-12));
    }

    @Test
    void posesSyncFromSharedArray() {
        TransformArray bodies = TransformArray.onHeap(3);
        int a = world.addDynamic(new SphereShape(1), Transform.IDENTITY);
        int b = world.addStatic(new BoxShape(Vec3.ONE), Transform.IDENTITY);
        int c = world.addDynamic(new SphereShape(1), Transform.ofTranslation(Vec3.of(7, 7, 7)));
        world.setPoseSource(a, 2);
        world.setPoseSource(b, 0);
        world.setPoseSource(c, -5);
        assertThat(world.poseSource(a)).isEqualTo(2);
        assertThat(world.poseSource(c)).isEqualTo(-1);
        Transform moved = Transform.of(Vec3.of(1, 2, 3), Quat.fromYawPitchRoll(0.1, 0.2, 0.3), Vec3.of(1, 2, 1));
        bodies.set(2, moved);
        bodies.set(0, Transform.ofTranslation(Vec3.of(-5, 0, 0)));
        world.syncPoses(bodies);
        assertThat(world.pose(a)).isEqualTo(moved);
        assertThat(world.pose(b).translation()).isEqualTo(Vec3.of(-5, 0, 0));
        assertThat(world.pose(c).translation()).isEqualTo(Vec3.of(7, 7, 7));
        world.update();
        RaycastHit hit = new RaycastHit();
        assertThat(world.newQuery().raycast(Vec3.of(-5, 10, 0), Vec3.of(0, -1, 0), 100, -1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(b);
        world.syncPoses(bodies); // nothing moved: the static tree stays as it is
        world.update();
        assertThat(world.newQuery().raycast(Vec3.of(-5, 10, 0), Vec3.of(0, -1, 0), 100, -1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(b);
        // Poses must be finite and scale every axis.
        Transform nan = Transform.ofTranslation(Vec3.of(Double.NaN, 0, 0));
        assertThatIllegalArgumentException().isThrownBy(() -> world.addDynamic(new SphereShape(1), nan));
        assertThatIllegalArgumentException().isThrownBy(() -> world.setPose(a, nan));
        Transform flat = Transform.of(Vec3.ZERO, Quat.IDENTITY, Vec3.of(1, 0, 1));
        assertThatIllegalArgumentException().isThrownBy(() -> world.setPose(a, flat));
        Transform infinite = Transform.of(Vec3.ZERO, Quat.IDENTITY, Vec3.of(1, Double.POSITIVE_INFINITY, 1));
        assertThatIllegalArgumentException().isThrownBy(() -> world.setPose(a, infinite));
        assertThatIllegalArgumentException().isThrownBy(() -> world.setPose(a, Transform.of(Vec3.ZERO, Quat.IDENTITY,
            Vec3.of(0, 1, 1))));
        assertThatIllegalArgumentException().isThrownBy(() -> world.setPose(a, Transform.of(Vec3.ZERO, Quat.IDENTITY,
            Vec3.of(1, 1, 0))));
        assertThat(world.size()).isEqualTo(3);
        world.setPoseSource(a, 3);
        assertThatThrownBy(() -> world.syncPoses(bodies)).isInstanceOf(IndexOutOfBoundsException.class);
    }

    @Test
    void staleAndFilteredColliders() {
        // A moving mesh: paired with spheres, never with the static heightfield (two concave shapes).
        double[] bump = new double[25];
        bump[12] = 0.5;
        HeightfieldShape ground = HeightfieldShape.of(5, 5, 1, 1, bump);
        int field = world.addStatic(ground, Transform.ofTranslation(Vec3.of(-2, 0, -2)));
        TriangleMeshShape flat = TriangleContactsTest.meshOf(HeightfieldShape.of(5, 5, 1, 1, new double[25]));
        int platform = world.addDynamic(flat, Transform.ofTranslation(Vec3.of(-2, 0.25, -2)));
        int ball = world.addDynamic(new SphereShape(1), Transform.ofTranslation(Vec3.of(0, 1, 0)));
        int hidden = world.addDynamic(new SphereShape(1), Transform.ofTranslation(Vec3.of(0, 2.5, 0)));
        world.setFilter(hidden, 2, 2);
        world.update();
        PairBuffer pairs = new PairBuffer();
        world.findPairs(pairs);
        assertThat(pairsOf(pairs)).isEqualTo(bruteForcePairs()).contains(platform + "-" + ball)
            .doesNotContain(field + "-" + platform, ball + "-" + hidden);
        assertThat(world.bounds(field).intersects(world.bounds(platform))).isTrue();
        assertThat(world.bounds(ball).intersects(world.bounds(hidden))).isTrue();
        CollisionQuery query = world.newQuery();
        RaycastHit hit = new RaycastHit();
        // The hidden sphere is skipped by a mask without its group.
        assertThat(query.raycast(Vec3.of(0, 10, 0), Vec3.of(0, -1, 0), 100, 1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(ball);
        Set<Integer> found = new TreeSet<>();
        query.overlap(new SphereShape(3), Transform.ofTranslation(Vec3.of(0, 2, 0)), 1, collider -> found.add(collider));
        assertThat(found).containsExactlyInAnyOrder(field, platform, ball);
        // Removed but not yet updated: still in the trees, skipped by queries, pose sync and pairs.
        world.setPoseSource(ball, 0);
        world.remove(ball);
        world.syncPoses(TransformArray.onHeap(1));
        assertThat(query.raycast(Vec3.of(0, 10, 0), Vec3.of(0, -1, 0), 100, 1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(field); // the bump at 0.5 is above the platform at 0.25
        found.clear();
        query.overlap(new SphereShape(3), Transform.ofTranslation(Vec3.of(0, 2, 0)), -1, collider -> found.add(collider));
        assertThat(found).containsExactlyInAnyOrder(field, platform, hidden);
        world.findPairs(pairs);
        assertThat(pairsOf(pairs)).doesNotContain(platform + "-" + ball);
    }

    // ---------------------------------------------------------------- bookkeeping

    @Test
    void colliderProperties() {
        int id = world.addDynamic(new SphereShape(1), Transform.IDENTITY);
        assertThat(world.material(id)).isZero();
        assertThat(world.userData(id)).isZero();
        world.setMaterial(id, 7);
        world.setUserData(id, 42);
        assertThat(world.material(id)).isEqualTo(7);
        assertThat(world.userData(id)).isEqualTo(42);
        assertThat(world.shape(id)).isEqualTo(new SphereShape(1));
        assertThat(world.isStatic(id)).isFalse();
        assertThat(world.contains(id)).isTrue();
        assertThat(world.contains(-1)).isFalse();
        assertThat(world.contains(5)).isFalse();
        world.remove(id);
        assertThat(world.contains(id)).isFalse();
        assertThat(world.size()).isZero();
        assertThatIllegalArgumentException().isThrownBy(() -> world.shape(id)).withMessage("No collider 0");
        assertThatIllegalArgumentException().isThrownBy(() -> world.shape(-1)).withMessage("No collider -1");
        assertThatIllegalArgumentException().isThrownBy(() -> world.shape(5000)).withMessage("No collider 5000");
        assertThatIllegalArgumentException().isThrownBy(() -> world.remove(id));
        // Growing past the initial capacity.
        for (int i = 0; i < 40; i++) {
            assertThat(world.addStatic(new SphereShape(1), Transform.ofTranslation(Vec3.of(3 * i, 0, 0)))).isEqualTo(
                i == 0 ? 1 << 24 : i); // slot 0 again, its second generation
        }
        assertThat(world.size()).isEqualTo(40);
        // No more than an id has index bits for (fewer here).
        world.maxColliders = 40;
        assertThatIllegalStateException().isThrownBy(() -> world.addDynamic(new SphereShape(1), Transform.IDENTITY))
            .withMessage("At most 40 colliders");
    }

    @Test
    void boundsMarginFindsNearbyPairs() {
        world.addDynamic(new SphereShape(1), Transform.IDENTITY);
        world.addDynamic(new SphereShape(1), Transform.ofTranslation(Vec3.of(2.1, 0, 0)));
        world.update();
        PairBuffer pairs = new PairBuffer();
        world.findPairs(pairs);
        assertThat(pairs.size()).isZero();
        world.setBoundsMargin(0.06);
        assertThat(world.boundsMargin()).isEqualTo(0.06);
        world.update();
        world.findPairs(pairs);
        assertThat(pairs.size()).isEqualTo(1);
        ContactManifolds manifolds = new ContactManifolds(1);
        manifolds.prepare(1);
        world.newQuery().collide(pairs, 0, 1, 0.12, manifolds);
        assertThat(manifolds.get(0, 0).distance()).isCloseTo(0.1, within(1e-9));
        assertThatIllegalArgumentException().isThrownBy(() -> world.setBoundsMargin(-1));
        assertThatIllegalArgumentException().isThrownBy(() -> world.setBoundsMargin(Double.NaN));
    }

    @Test
    void pairBuffer() {
        PairBuffer pairs = new PairBuffer(0);
        for (int i = 0; i < 10; i++) {
            pairs.add(i, i + 1);
        }
        assertThat(pairs.size()).isEqualTo(10);
        assertThat(pairs.first(9)).isEqualTo(9);
        assertThat(pairs.second(9)).isEqualTo(10);
        assertThatThrownBy(() -> pairs.first(10)).isInstanceOf(IndexOutOfBoundsException.class);
        assertThatThrownBy(() -> pairs.second(-1)).isInstanceOf(IndexOutOfBoundsException.class);
        pairs.clear();
        assertThat(pairs.size()).isZero();
    }
}
