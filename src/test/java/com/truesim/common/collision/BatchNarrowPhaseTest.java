package com.truesim.common.collision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.random.Rng;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class BatchNarrowPhaseTest {

    private static final double MAX_DISTANCE = 0.05;

    private final CollisionWorld world = new CollisionWorld();
    private final PairBuffer pairs = new PairBuffer();

    /** A heightfield, a moving mesh platform and a crowd of convex shapes over them. */
    private void populate() {
        Rng rng = Rng.seeded(21);
        world.addStatic(HeightfieldShapeTest.hills(16, 16), Transform.IDENTITY);
        world.addDynamic(TriangleMeshShapeTest.grid(6), Transform.ofTranslation(Vec3.of(4, 1.2, 4)));
        for (int i = 0; i < 80; i++) {
            ConvexShape shape = switch (i % 3) {
                case 0 -> new SphereShape(rng.nextDouble(0.3, 0.9));
                case 1 -> new BoxShape(Vec3.of(rng.nextDouble(0.2, 0.7), rng.nextDouble(0.2, 0.7), rng.nextDouble(0.2, 0.7)));
                default -> new CapsuleShape(rng.nextDouble(0, 0.6), rng.nextDouble(0.2, 0.5));
            };
            world.addDynamic(shape, Transform.of(
                Vec3.of(rng.nextDouble(0, 30), rng.nextDouble(-1, 3), rng.nextDouble(0, 22)), rng.nextRotation()));
        }
        world.update();
        world.findPairs(pairs);
    }

    @Test
    void matchesPairwiseQueries() {
        populate();
        ContactManifolds manifolds = new ContactManifolds(8);
        manifolds.prepare(pairs.size());
        world.newQuery().collide(pairs, 0, pairs.size(), MAX_DISTANCE, manifolds);
        assertThat(manifolds.size()).isEqualTo(pairs.size());
        NarrowPhase narrow = new NarrowPhase();
        Contact single = new Contact();
        ContactBuffer buffer = new ContactBuffer(8);
        int meshPairs = 0;
        int flippedPairs = 0;
        for (int i = 0; i < pairs.size(); i++) {
            int a = pairs.first(i);
            int b = pairs.second(i);
            Shape shapeA = world.shape(a);
            Shape shapeB = world.shape(b);
            if (shapeA instanceof ConvexShape ca && shapeB instanceof ConvexShape cb) {
                double distance = narrow.query(ca, world.pose(a), cb, world.pose(b), single);
                assertThat(manifolds.count(i)).isEqualTo(distance <= MAX_DISTANCE ? 1 : 0);
                if (distance <= MAX_DISTANCE) {
                    Contact c = manifolds.get(i, 0);
                    assertThat(c.distance()).isEqualTo(distance);
                    assertThat(c.normal()).isEqualTo(single.normal());
                }
                continue;
            }
            // One side is a mesh or heightfield: compare by triangle, with the normal from A to B.
            boolean flipped = shapeA instanceof ConcaveShape;
            int convex = flipped ? b : a;
            int concave = flipped ? a : b;
            int count = narrow.contacts((ConvexShape) world.shape(convex), world.pose(convex),
                (ConcaveShape) world.shape(concave), world.pose(concave), MAX_DISTANCE, buffer);
            assertThat(manifolds.count(i)).isEqualTo(count);
            meshPairs += count > 0 ? 1 : 0;
            flippedPairs += flipped && count > 0 ? 1 : 0;
            Map<Integer, Contact> expected = new HashMap<>();
            for (int k = 0; k < count; k++) {
                expected.put(buffer.get(k).featureB(), buffer.get(k));
            }
            for (int k = 0; k < count; k++) {
                Contact c = manifolds.get(i, k);
                Contact e = expected.get(flipped ? c.featureA() : c.featureB());
                assertThat(c.distance()).isEqualTo(e.distance());
                double sign = flipped ? -1 : 1;
                assertThat(c.normalX()).isEqualTo(sign * e.normalX());
                assertThat(c.normalY()).isEqualTo(sign * e.normalY());
                assertThat(c.normalZ()).isEqualTo(sign * e.normalZ());
                assertThat(flipped ? c.pointB() : c.pointA()).isEqualTo(e.pointA());
                assertThat(flipped ? c.materialA() : c.materialB()).isEqualTo(e.materialB());
                assertThat(flipped ? c.featureB() : c.featureA()).isEqualTo(Contact.NONE);
            }
        }
        assertThat(meshPairs).isGreaterThan(5);
        assertThat(flippedPairs).isPositive();
        assertThat(manifolds.totalContacts()).isGreaterThan(pairs.size() / 4);
    }

    @Test
    void threadsAndRangesGiveIdenticalResults() throws InterruptedException {
        populate();
        ContactManifolds reference = new ContactManifolds(4);
        reference.prepare(pairs.size());
        world.newQuery().collide(pairs, 0, pairs.size(), MAX_DISTANCE, reference);
        ContactManifolds parallel = new ContactManifolds(4);
        parallel.prepare(pairs.size());
        // Four workers, each with its own query, take interleaved chunks of three pairs, back to front.
        List<Thread> workers = new ArrayList<>();
        for (int w = 0; w < 4; w++) {
            int worker = w;
            CollisionQuery query = world.newQuery();
            workers.add(Thread.ofPlatform().start(() -> {
                int chunks = (pairs.size() + 2) / 3;
                for (int chunk = chunks - 1 - worker; chunk >= 0; chunk -= 4) {
                    query.collide(pairs, 3 * chunk, Math.min(pairs.size(), 3 * chunk + 3), MAX_DISTANCE, parallel);
                }
            }));
        }
        for (Thread worker : workers) {
            worker.join();
        }
        for (int i = 0; i < pairs.size(); i++) {
            assertThat(parallel.count(i)).isEqualTo(reference.count(i));
            for (int k = 0; k < reference.count(i); k++) {
                Contact r = reference.get(i, k);
                Contact p = parallel.get(i, k);
                assertThat(p.distance()).isEqualTo(r.distance());
                assertThat(p.normal()).isEqualTo(r.normal());
                assertThat(p.pointA()).isEqualTo(r.pointA());
                assertThat(p.pointB()).isEqualTo(r.pointB());
                assertThat(p.featureB()).isEqualTo(r.featureB());
            }
        }
    }

    @Test
    void stalePairsAndBounds() {
        int a = world.addDynamic(new SphereShape(1), Transform.IDENTITY);
        int b = world.addDynamic(new SphereShape(1), Transform.ofTranslation(Vec3.of(1.5, 0, 0)));
        world.update();
        world.findPairs(pairs);
        assertThat(pairs.size()).isEqualTo(1);
        CollisionQuery query = world.newQuery();
        ContactManifolds one = new ContactManifolds(1);
        one.prepare(1);
        query.collide(pairs, 0, 1, 0, one);
        assertThat(one.count(0)).isEqualTo(1);
        assertThat(one.get(0, 0).distance()).isCloseTo(-0.5, within(1e-12));
        assertThat(one.contactsPerPair()).isEqualTo(1);
        // A different number of contacts per pair, and a pair whose collider was removed since.
        ContactManifolds four = new ContactManifolds(4);
        four.prepare(1);
        world.remove(b);
        query.collide(pairs, 0, 1, 0, four);
        assertThat(four.count(0)).isZero();
        assertThat(four.totalContacts()).isZero();
        world.remove(a);
        query.collide(pairs, 0, 1, 0, four);
        assertThat(four.count(0)).isZero();
        // Ranges must lie within both the pairs and the prepared manifolds.
        assertThatThrownBy(() -> query.collide(pairs, 0, 2, 0, four)).isInstanceOf(IndexOutOfBoundsException.class);
        four.prepare(0);
        assertThatThrownBy(() -> query.collide(pairs, 0, 1, 0, four)).isInstanceOf(IndexOutOfBoundsException.class);
        assertThatThrownBy(() -> four.count(0)).isInstanceOf(IndexOutOfBoundsException.class);
        assertThatThrownBy(() -> one.get(0, 1)).isInstanceOf(IndexOutOfBoundsException.class);
        assertThatThrownBy(() -> one.get(0, -1)).isInstanceOf(IndexOutOfBoundsException.class);
        assertThatThrownBy(() -> one.count(-1)).isInstanceOf(IndexOutOfBoundsException.class);
        assertThatIllegalArgumentException().isThrownBy(() -> new ContactManifolds(0));
        // Preparing grows only past the largest size so far, and clears counts.
        one.prepare(100);
        assertThat(one.size()).isEqualTo(100);
        one.prepare(50);
        assertThat(one.count(49)).isZero();
    }
}
