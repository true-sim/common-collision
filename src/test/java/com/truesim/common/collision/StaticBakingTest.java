package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;

import com.truesim.common.math.Quat;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.random.Rng;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import org.junit.jupiter.api.Test;

class StaticBakingTest {

    private static final Transform FIELD_POSE = Transform.of(Vec3.of(-4, -1, -3), Quat.fromAxisAngle(Vec3.UP, 0.2));
    private static final Transform TRACK_POSE =
        Transform.of(Vec3.of(30, 0.5, 2), Quat.fromYawPitchRoll(0.4, 0.05, 0), Vec3.of(1.5, 1, 1.5));

    /** The same scene, with its heightfield and mesh baked or not. Collider ids are the same in both. */
    private static CollisionWorld scene(boolean bake) {
        CollisionWorld world = new CollisionWorld();
        world.addStatic(HeightfieldShapeTest.hills(16, 12), FIELD_POSE, bake);
        world.addStatic(TriangleMeshShapeTest.grid(10), TRACK_POSE, bake);
        world.addStatic(new BoxShape(Vec3.of(1, 3, 1)), Transform.ofTranslation(Vec3.of(20, 1, 5)));
        Rng rng = Rng.seeded(31);
        for (int i = 0; i < 40; i++) {
            ConvexShape shape = i % 2 == 0 ? new SphereShape(rng.nextDouble(0.3, 0.8))
                : new BoxShape(Vec3.of(rng.nextDouble(0.2, 0.6), rng.nextDouble(0.2, 0.6), rng.nextDouble(0.2, 0.6)));
            world.addDynamic(shape, Transform.of(
                Vec3.of(rng.nextDouble(-4, 45), rng.nextDouble(-1.5, 2.5), rng.nextDouble(-3, 18)), rng.nextRotation()));
        }
        world.setFilter(4, 2, -1); // one sphere outside group 1
        world.update();
        return world;
    }

    private static Set<String> pairs(CollisionWorld world) {
        PairBuffer pairs = new PairBuffer();
        world.findPairs(pairs);
        Set<String> set = new TreeSet<>();
        for (int i = 0; i < pairs.size(); i++) {
            set.add(pairs.first(i) + "-" + pairs.second(i));
        }
        assertThat(set).hasSize(pairs.size()); // a baked collider is paired once, not once per triangle
        return set;
    }

    private static void assertSameQueries(CollisionWorld plain, CollisionWorld baked, Rng rng) {
        CollisionQuery a = plain.newQuery();
        CollisionQuery b = baked.newQuery();
        RaycastHit hitA = new RaycastHit();
        RaycastHit hitB = new RaycastHit();
        int hits = 0;
        for (int i = 0; i < 600; i++) {
            Vec3 origin = Vec3.of(rng.nextDouble(-6, 50), rng.nextDouble(2, 8), rng.nextDouble(-6, 20));
            Vec3 direction = i % 3 == 0 ? Vec3.of(0, -1, 0) : rng.nextVec3OnSphere();
            int mask = i % 7 == 0 ? 2 : -1;
            boolean foundA = a.raycast(origin, direction, 40, mask, hitA);
            assertThat(b.raycast(origin, direction, 40, mask, hitB)).isEqualTo(foundA);
            if (foundA) {
                hits++;
                assertThat(hitB.distance()).isCloseTo(hitA.distance(), within(1e-9));
                assertThat(hitB.collider()).isEqualTo(hitA.collider());
                if (hitB.triangle() == hitA.triangle()) {
                    // On a shared edge either triangle may be reported; otherwise everything matches.
                    assertThat(hitB.material()).isEqualTo(hitA.material());
                    assertThat(hitB.normal().dot(hitA.normal())).isCloseTo(1, within(1e-9));
                    assertThat(hitB.point().sub(hitA.point()).length()).isCloseTo(0, within(1e-9));
                }
            }
        }
        assertThat(hits).isGreaterThan(100);
        for (int i = 0; i < 200; i++) {
            ConvexShape shape = i % 2 == 0 ? new SphereShape(rng.nextDouble(0.3, 2)) : new BoxShape(Vec3.of(1, 0.5, 2));
            Transform pose = Transform.of(
                Vec3.of(rng.nextDouble(-6, 50), rng.nextDouble(-1.5, 2.5), rng.nextDouble(-6, 20)), rng.nextRotation());
            Set<Integer> foundA = new TreeSet<>();
            Set<Integer> foundB = new TreeSet<>();
            int countA = a.overlap(shape, pose, -1, foundA::add);
            int countB = b.overlap(shape, pose, -1, collider -> {
                assertThat(foundB.add(collider)).isTrue(); // each collider once
                return true;
            });
            assertThat(foundB).isEqualTo(foundA);
            assertThat(countB).isEqualTo(countA);
        }
        assertSameContacts(plain, baked);
    }

    /**
     * Baked geometry pairs by the bounds of single triangles, which are tighter than a whole mesh's: its pairs are a
     * subset, and the pairs it leaves out have no contacts. Every other pair has the same contacts.
     */
    private static void assertSameContacts(CollisionWorld plain, CollisionWorld baked) {
        Set<String> bakedPairs = pairs(baked);
        assertThat(pairs(plain)).containsAll(bakedPairs);
        java.util.Map<String, java.util.List<Double>> plainContacts = contacts(plain);
        java.util.Map<String, java.util.List<Double>> bakedContacts = contacts(baked);
        for (var entry : plainContacts.entrySet()) {
            if (bakedPairs.contains(entry.getKey())) {
                assertThat(bakedContacts.get(entry.getKey())).isEqualTo(entry.getValue());
            } else {
                assertThat(entry.getValue()).as("pair %s", entry.getKey()).isEmpty();
            }
        }
    }

    /** Each pair's contact distances, sorted. */
    private static java.util.Map<String, java.util.List<Double>> contacts(CollisionWorld world) {
        PairBuffer pairs = new PairBuffer();
        world.findPairs(pairs);
        ContactManifolds manifolds = new ContactManifolds(64);
        manifolds.prepare(pairs.size());
        world.newQuery().collide(pairs, 0, pairs.size(), 0, manifolds);
        java.util.Map<String, java.util.List<Double>> result = new java.util.HashMap<>();
        for (int i = 0; i < pairs.size(); i++) {
            java.util.List<Double> distances = new java.util.ArrayList<>();
            for (int k = 0; k < manifolds.count(i); k++) {
                distances.add(manifolds.get(i, k).distance());
            }
            distances.sort(null);
            result.put(pairs.first(i) + "-" + pairs.second(i), distances);
        }
        return result;
    }

    @Test
    void bakedAndUnbakedGiveTheSameResults() {
        CollisionWorld plain = scene(false);
        CollisionWorld baked = scene(true);
        assertThat(baked.isBaked(0)).isTrue();
        assertThat(baked.isBaked(1)).isTrue();
        assertThat(baked.isBaked(2)).isFalse();
        assertThat(plain.isBaked(0)).isFalse();
        // The static tree holds every triangle of the baked shapes, plus the box.
        assertThat(baked.staticColliders).hasSize(2 * 16 * 12 + 2 * 10 * 10 + 1);
        assertSameQueries(plain, baked, Rng.seeded(1));
        // Moving baked geometry rebuilds the static tree from the moved triangles.
        Transform moved = TRACK_POSE.withTranslation(Vec3.of(26, 1, 4));
        plain.setPose(1, moved);
        baked.setPose(1, moved);
        plain.update();
        baked.update();
        assertSameQueries(plain, baked, Rng.seeded(2));
        // Exactly onto a grid vertex of the moved track: same triangle mesh, same answer.
        RaycastHit hit = new RaycastHit();
        Vec3 vertex = moved.transformPoint(Vec3.of(5, 0.3 * Math.sin(5) * Math.cos(5), 5));
        assertThat(baked.newQuery().raycast(vertex.add(Vec3.of(0, 5, 0)), Vec3.of(0, -1, 0), 10, -1, hit)).isTrue();
        assertThat(hit.collider()).isEqualTo(1);
        assertThat(hit.distance()).isCloseTo(5, within(1e-9));
    }

    @Test
    void removedAndFilteredBakedColliders() {
        CollisionWorld world = scene(true);
        CollisionQuery query = world.newQuery();
        RaycastHit hit = new RaycastHit();
        Vec3 above = FIELD_POSE.transformPoint(Vec3.of(10, 8, 9));
        assertThat(query.raycast(above, Vec3.of(0, -1, 0), 20, -1, hit)).isTrue();
        assertThat(hit.collider()).isZero();
        // Filtered out by the mask.
        world.setFilter(0, 4, -1);
        assertThat(query.raycast(above, Vec3.of(0, -1, 0), 20, 1, hit)).isFalse();
        Set<Integer> found = new TreeSet<>();
        query.overlap(new SphereShape(3), Transform.ofTranslation(above.add(Vec3.of(0, -8, 0))), 1, found::add);
        assertThat(found).doesNotContain(0);
        // Stopping at the first baked collider found.
        assertThat(query.overlap(new SphereShape(3), Transform.ofTranslation(above.add(Vec3.of(0, -8, 0))), 4,
            collider -> false)).isEqualTo(1);
        // Removed, before and after the next update.
        world.remove(0);
        assertThat(query.raycast(above, Vec3.of(0, -1, 0), 20, -1, hit)).isFalse();
        found.clear();
        query.overlap(new SphereShape(3), Transform.ofTranslation(above.add(Vec3.of(0, -8, 0))), -1, found::add);
        assertThat(found).doesNotContain(0);
        world.update();
        assertThat(world.staticColliders).hasSize(2 * 10 * 10 + 1);
        assertThat(query.raycast(above, Vec3.of(0, -1, 0), 20, -1, hit)).isFalse();
        // A miss beyond the maximum distance.
        assertThat(query.raycast(TRACK_POSE.transformPoint(Vec3.of(5, 5, 5)), Vec3.of(0, -1, 0), 1, -1, hit)).isFalse();
    }

    @Test
    void onlyConcaveShapesBake() {
        CollisionWorld world = new CollisionWorld();
        assertThatIllegalArgumentException()
            .isThrownBy(() -> world.addStatic(new SphereShape(1), Transform.IDENTITY, true))
            .withMessageContaining("Only meshes and heightfields can be baked");
        int plain = world.addStatic(TriangleMeshShapeTest.grid(2), Transform.IDENTITY, false);
        assertThat(world.isBaked(plain)).isFalse();
    }

    @Test
    void slantedBakedTriangle() {
        // One triangle on the slope y = x, facing up and to -x.
        CollisionWorld world = new CollisionWorld();
        int ramp = world.addStatic(TriangleMeshShape.of(List.of(Vec3.ZERO, Vec3.of(10, 10, 1), Vec3.of(10, 10, 0)),
            new int[] {0, 1, 2}), Transform.IDENTITY, true);
        world.update();
        CollisionQuery query = world.newQuery();
        RaycastHit hit = new RaycastHit();
        // Enters the triangle's box at distance 10 but meets the triangle at 12: beyond a maximum of 11.
        assertThat(query.raycast(Vec3.of(8, 20, 0.5), Vec3.of(0, -1, 0), 11, -1, hit)).isFalse();
        assertThat(query.raycast(Vec3.of(8, 20, 0.5), Vec3.of(0, -1, 0), 13, -1, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(12, within(1e-12));
        assertThat(hit.collider()).isEqualTo(ramp);
        assertThat(hit.normal().dot(Vec3.of(-1, 1, 0).normalize())).isCloseTo(1, within(1e-12));
        // From underneath: the normal still faces the ray.
        assertThat(query.raycast(Vec3.of(8, -5, 0.5), Vec3.of(0, 1, 0), 20, -1, hit)).isTrue();
        assertThat(hit.distance()).isCloseTo(13, within(1e-12));
        assertThat(hit.normal().dot(Vec3.of(1, -1, 0).normalize())).isCloseTo(1, within(1e-12));
    }
}
