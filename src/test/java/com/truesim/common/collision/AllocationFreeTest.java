package com.truesim.common.collision;

import static com.truesim.testing.AllocationAssertions.assertAllocationFree;

import com.truesim.common.math.Quat;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Verifies that narrow-phase queries do not allocate once the JIT has compiled them. */
class AllocationFreeTest {

    private final NarrowPhase narrow = new NarrowPhase();
    private final Contact contact = new Contact();
    private double sink;

    @Test
    void separatedQueries() {
        CapsuleShape capsule = new CapsuleShape(1, 0.5);
        ConvexHullShape hull = ConvexHullShape.of(List.of(Vec3.of(-1, 0, 0), Vec3.of(1, 0, 0), Vec3.of(0, 1, 0), Vec3.of(0, 0, 1)));
        Transform ta = Transform.of(Vec3.ZERO, Quat.fromYawPitchRoll(0.3, 0.2, 0.1));
        double[] x = {0};
        Quat rb = Quat.fromYawPitchRoll(-0.4, 0.5, 0.9);
        assertAllocationFree(() -> {
            x[0] += 1e-9;
            Transform tb = Transform.of(Vec3.of(3 + x[0], 1, 0.5), rb);
            sink += narrow.query(capsule, ta, hull, tb, contact) + contact.normalX();
            sink += narrow.overlaps(capsule, ta, hull, tb) ? 1 : 0;
        });
    }

    @Test
    void penetratingQueries() {
        BoxShape box = new BoxShape(Vec3.of(1, 1, 1));
        Transform ta = Transform.of(Vec3.ZERO, Quat.fromYawPitchRoll(0.3, 0.2, 0.1));
        double[] x = {0};
        Quat rb = Quat.fromYawPitchRoll(-0.4, 0.5, 0.9);
        assertAllocationFree(() -> {
            x[0] += 1e-9;
            Transform tb = Transform.of(Vec3.of(1.2 + x[0], 0.9, 0.7), rb);
            sink += narrow.query(box, ta, box, tb, contact) + contact.normalY();
        });
    }

    @Test
    void scaledRoundShapes() {
        SphereShape sphere = new SphereShape(1);
        Transform ellipsoid = Transform.of(Vec3.ZERO, Quat.IDENTITY, Vec3.of(3, 1, 1));
        double[] x = {0};
        Quat rb = Quat.IDENTITY;
        assertAllocationFree(() -> {
            x[0] += 1e-9;
            Transform tb = Transform.of(Vec3.of(2 + x[0], 2, 0), rb);
            sink += narrow.query(sphere, ellipsoid, sphere, tb, contact);
        });
    }

    @Test
    void meshQueries() {
        TriangleMeshShape mesh = TriangleMeshShapeTest.grid(32);
        int[] materials = new int[4];
        com.truesim.common.math.spatial.Bvh.Visitor visitor = triangle -> {
            materials[mesh.material(triangle)]++;
            return true;
        };
        double[] x = {0};
        assertAllocationFree(() -> {
            x[0] = x[0] > 30 ? 0 : x[0] + 0.37;
            sink += mesh.queryAabb(x[0], -1, 3, x[0] + 1.5, 1, 4.5, visitor);
        });
    }

    @Test
    void heightfieldQueries() {
        HeightfieldShape field = HeightfieldShapeTest.hills(64, 64);
        int[] materials = new int[3];
        com.truesim.common.math.spatial.Bvh.Visitor visitor = triangle -> {
            materials[field.material(triangle)]++;
            return true;
        };
        double[] x = {0};
        assertAllocationFree(() -> {
            x[0] = x[0] > 120 ? 0 : x[0] + 0.37;
            sink += field.queryAabb(x[0], -0.5, 30, x[0] + 3, 0.5, 33, visitor);
        });
    }

    private final RayCaster caster = new RayCaster();
    private final RaycastHit rayHit = new RaycastHit();

    @Test
    void raycasts() {
        BoxShape box = new BoxShape(Vec3.of(1, 2, 3));
        CapsuleShape capsule = new CapsuleShape(1, 0.5);
        ConvexHullShape hull = ConvexHullShape.of(List.of(Vec3.of(-1, 0, 0), Vec3.of(1, 0, 0), Vec3.of(0, 1, 0), Vec3.of(0, 0, 1)));
        TriangleMeshShape mesh = TriangleMeshShapeTest.grid(16);
        HeightfieldShape field = HeightfieldShapeTest.hills(32, 32);
        Quat rotation = Quat.fromYawPitchRoll(0.3, 0.2, 0.1);
        double[] x = {0};
        assertAllocationFree(() -> {
            x[0] = x[0] > 10 ? 0 : x[0] + 0.01;
            Transform pose = Transform.of(Vec3.of(x[0], 0, 0), rotation);
            caster.cast(box, pose, Vec3.of(x[0], 10, 0.1), Vec3.of(0.01, -1, 0), 100, rayHit);
            sink += rayHit.distance();
            caster.cast(capsule, pose, Vec3.of(x[0], 10, 0.1), Vec3.of(0.01, -1, 0), 100, rayHit);
            sink += rayHit.normalY();
            caster.cast(hull, pose, Vec3.of(x[0], 10, 0.1), Vec3.of(0.01, -1, 0), 100, rayHit);
            sink += rayHit.distance();
            caster.cast(mesh, pose, Vec3.of(x[0] + 3, 10, 4), Vec3.of(0.01, -1, 0.02), 100, rayHit);
            sink += rayHit.material();
            caster.cast(field, Transform.IDENTITY, Vec3.of(x[0] + 1, 10, 4), Vec3.of(1, -0.3, 0.5), 100, rayHit);
            sink += rayHit.triangle();
        });
    }

    private final ContactBuffer buffer = new ContactBuffer(32);

    @Test
    void triangleContacts() {
        BoxShape box = new BoxShape(Vec3.of(0.5, 0.4, 0.8));
        SphereShape ball = new SphereShape(0.6);
        TriangleMeshShape mesh = TriangleMeshShapeTest.grid(16);
        HeightfieldShape field = HeightfieldShapeTest.hills(32, 32);
        Quat rotation = Quat.fromYawPitchRoll(0.3, 0.2, 0.1);
        double[] x = {0};
        assertAllocationFree(() -> {
            x[0] = x[0] > 10 ? 0 : x[0] + 0.01;
            Transform pose = Transform.of(Vec3.of(3 + x[0], 0.2, 4), rotation);
            sink += narrow.contacts(box, pose, mesh, Transform.IDENTITY, 0.05, buffer);
            sink += narrow.contacts(ball, Transform.of(Vec3.of(5 + x[0], 0.5, 9), rotation), field,
                Transform.ofTranslation(Vec3.of(0, 0.1, 0)), 0.05, buffer);
        });
    }

    @Test
    void worldSteps() {
        CollisionWorld world = new CollisionWorld();
        world.addStatic(HeightfieldShapeTest.hills(32, 32), Transform.IDENTITY);
        com.truesim.common.math.TransformArray bodies = com.truesim.common.math.TransformArray.onHeap(40);
        for (int i = 0; i < 40; i++) {
            int id = world.addDynamic(i % 2 == 0 ? new SphereShape(0.5) : new BoxShape(Vec3.of(0.4, 0.3, 0.5)),
                Transform.IDENTITY);
            world.setPoseSource(id, i);
            bodies.set(i, Transform.ofTranslation(Vec3.of(1.5 * i % 60, 1, 2 + i % 7)));
        }
        world.update();
        CollisionQuery query = world.newQuery();
        PairBuffer pairs = new PairBuffer(256);
        SphereShape probe = new SphereShape(1.5);
        int[] hits = {0};
        ColliderVisitor counter = collider -> {
            hits[0]++;
            return true;
        };
        double[] x = {0};
        assertAllocationFree(() -> {
            x[0] = x[0] > 30 ? 0 : x[0] + 0.01;
            world.syncPoses(bodies);
            world.update();
            world.findPairs(pairs);
            query.raycast(Vec3.of(x[0], 10, 4), Vec3.of(0, -1, 0.01), 100, -1, rayHit);
            sink += pairs.size() + rayHit.distance();
        });
        assertAllocationFree(() -> {
            x[0] = x[0] > 30 ? 0 : x[0] + 0.01;
            sink += query.overlap(probe, Transform.ofTranslation(Vec3.of(x[0], 1, 4)), -1, counter);
        });
    }
}
