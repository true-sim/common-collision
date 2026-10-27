package com.truesim.common.collision;

import com.truesim.annotations.AllowAllocation;
import com.truesim.common.math.JitInlining;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Intersections;
import com.truesim.common.math.spatial.Bvh;
import com.truesim.common.math.spatial.RayHit;
import java.util.Arrays;
import java.util.Objects;

/**
 * Queries against a {@link CollisionWorld} from one thread: raycasts, overlaps, and the narrow phase over ranges of
 * broad-phase pairs ({@link #collide}). Holds the working storage, so
 * queries allocate nothing; create one per thread with {@link CollisionWorld#newQuery()}. Queries see the world as of
 * its last {@link CollisionWorld#update()}; several threads may query concurrently while nobody changes the world.
 */
public final class CollisionQuery {

    static {
        // Inline the thin query methods into callers so the vectors and transforms they create need no allocation.
        JitInlining.include(
            "+com/truesim/common/collision/CollisionQuery.raycast", "+com/truesim/common/collision/CollisionQuery.overlap");
    }

    private final CollisionWorld world;
    private final RayCaster caster = new RayCaster();
    private final NarrowPhase narrow = new NarrowPhase();
    private final RaycastHit candidate = new RaycastHit();
    private final RaycastHit best = new RaycastHit();
    private final RayHit treeHit = new RayHit();
    private final ContactBuffer probe = new ContactBuffer(1);
    private final Contact pairContact = new Contact();
    // Baked triangles: world-space corners, and each baked collider reported once per overlap query.
    private static final double[] IDENTITY_POSE = {0, 0, 0, 0, 0, 0, 1, 1, 1, 1};
    private final TriangleCore triangle = new TriangleCore();
    private final double[] localTriangle = new double[9];
    private final Contact triangleContact = new Contact();
    private long[] seen = new long[0];
    private long stamp;
    private ContactBuffer pairContacts = new ContactBuffer(4);
    private final double[] queryPose = new double[CollisionWorld.POSE];
    private final double[] queryLocal = new double[6];
    private final double[] queryBounds = new double[6];
    private final double[] support = new double[3];
    private final Bvh.RayTest staticRay;
    private final Bvh.RayTest dynamicRay;
    private final Bvh.Visitor staticOverlap;
    private final Bvh.Visitor dynamicOverlap;
    private final Bvh.Visitor dynamicPairs;
    private final Bvh.Visitor staticPairs;
    private PairBuffer pairTarget;
    private int pairSlot;
    private int pairCollider;
    private int mask;
    private ConvexShape queryShape;
    private ColliderVisitor visitor;
    private int found;
    private boolean stopped;

    @AllowAllocation("one ray test and visitor each per query context")
    CollisionQuery(CollisionWorld world) {
        this.world = world;
        staticRay = this::staticRay;
        dynamicRay = this::dynamicRay;
        staticOverlap = this::staticOverlap;
        dynamicOverlap = this::dynamicOverlap;
        dynamicPairs = this::dynamicPair;
        staticPairs = this::staticPair;
    }

    // ---------------------------------------------------------------- raycasts

    /**
     * The nearest collider hit by the ray {@code origin + t * direction}, {@code 0 <= t <= maxDistance}, among those
     * whose group shares a bit with {@code mask}. The hit names the collider ({@link RaycastHit#collider()}).
     */
    public boolean raycast(Vec3 origin, Vec3 direction, double maxDistance, int mask, RaycastHit hit) {
        return raycast(origin.x(), origin.y(), origin.z(), direction.x(), direction.y(), direction.z(), maxDistance, mask,
            hit);
    }

    public boolean raycast(double ox, double oy, double oz, double dx, double dy, double dz, double maxDistance,
        int mask, RaycastHit hit) {
        this.mask = mask;
        best.distance = Double.POSITIVE_INFINITY;
        best.collider = RaycastHit.NONE;
        world.staticTree.raycast(ox, oy, oz, dx, dy, dz, maxDistance, staticRay, treeHit);
        world.dynamicTree.raycast(ox, oy, oz, dx, dy, dz, Math.min(maxDistance, best.distance), dynamicRay, treeHit);
        hit.copyFrom(best);
        return best.collider != RaycastHit.NONE;
    }

    private double staticRay(int slot, double ox, double oy, double oz, double dx, double dy, double dz, double max) {
        int t = world.staticTriangles[slot];
        return t < 0
            ? ray(world.staticColliders[slot], ox, oy, oz, dx, dy, dz, max)
            : rayTriangle(world.staticColliders[slot], t, ox, oy, oz, dx, dy, dz, max);
    }

    /** A ray against one triangle of a baked collider, already in world space. */
    private double rayTriangle(int collider, int t, double ox, double oy, double oz, double dx, double dy, double dz,
        double max) {
        if (!world.isLive(collider) || (world.groups[collider] & mask) == 0) {
            return Double.POSITIVE_INFINITY;
        }
        double[] c = triangle.corners;
        world.worldTriangle(collider, t, c, localTriangle);
        double distance = Intersections.rayTriangle(ox, oy, oz, dx, dy, dz, c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
            c[8]);
        if (distance == Double.POSITIVE_INFINITY || distance > max) {
            return Double.POSITIVE_INFINITY;
        }
        double e1x = c[3] - c[0];
        double e1y = c[4] - c[1];
        double e1z = c[5] - c[2];
        double e2x = c[6] - c[0];
        double e2y = c[7] - c[1];
        double e2z = c[8] - c[2];
        double nx = e1y * e2z - e1z * e2y;
        double ny = e1z * e2x - e1x * e2z;
        double nz = e1x * e2y - e1y * e2x;
        // Facing the ray, unit length.
        double scale = (nx * dx + ny * dy + nz * dz > 0 ? -1 : 1) / Math.sqrt(nx * nx + ny * ny + nz * nz);
        best.distance = distance;
        best.pointX = ox + distance * dx;
        best.pointY = oy + distance * dy;
        best.pointZ = oz + distance * dz;
        best.normalX = nx * scale;
        best.normalY = ny * scale;
        best.normalZ = nz * scale;
        best.triangle = t;
        best.material = ((ConcaveShape) world.shapes[collider]).material(t);
        best.collider = collider;
        return distance;
    }

    private double dynamicRay(int slot, double ox, double oy, double oz, double dx, double dy, double dz, double max) {
        return ray(world.dynamicColliders[slot], ox, oy, oz, dx, dy, dz, max);
    }

    private double ray(int collider, double ox, double oy, double oz, double dx, double dy, double dz, double max) {
        if (!world.isLive(collider) || (world.groups[collider] & mask) == 0) {
            return Double.POSITIVE_INFINITY;
        }
        double[] p = world.poses;
        int i = CollisionWorld.POSE * collider;
        if (!caster.castPlaced(world.shapes[collider], p[i], p[i + 1], p[i + 2], p[i + 3], p[i + 4], p[i + 5], p[i + 6],
            p[i + 7], p[i + 8], p[i + 9], ox, oy, oz, dx, dy, dz, max, candidate)) {
            return Double.POSITIVE_INFINITY;
        }
        // The trees pass the best distance so far as the limit, so a hit is always the new best.
        best.copyFrom(candidate);
        best.collider = collider;
        return candidate.distance;
    }

    // ---------------------------------------------------------------- overlaps

    /**
     * Visits every collider that touches or overlaps the convex shape at {@code pose}, among those whose group shares
     * a bit with {@code mask}. Returns how many were visited; the visitor returns {@code false} to stop.
     */
    public int overlap(ConvexShape shape, Transform pose, int mask, ColliderVisitor visitor) {
        double[] q = queryPose;
        q[0] = pose.translation().x();
        q[1] = pose.translation().y();
        q[2] = pose.translation().z();
        q[3] = pose.rotation().x();
        q[4] = pose.rotation().y();
        q[5] = pose.rotation().z();
        q[6] = pose.rotation().w();
        q[7] = pose.scale().x();
        q[8] = pose.scale().y();
        q[9] = pose.scale().z();
        return overlapPlaced(shape, mask, visitor);
    }

    private int overlapPlaced(ConvexShape shape, int mask, ColliderVisitor visitor) {
        this.mask = mask;
        this.queryShape = shape;
        this.visitor = visitor;
        found = 0;
        stopped = false;
        stamp++;
        if (seen.length < world.capacity()) {
            growSeen(world.capacity());
        }
        // Local bounds from the support function: exact, and no allocation.
        double margin = shape.margin();
        shape.coreSupport(-1, 0, 0, support, 0);
        queryLocal[0] = support[0] - margin;
        shape.coreSupport(0, -1, 0, support, 0);
        queryLocal[1] = support[1] - margin;
        shape.coreSupport(0, 0, -1, support, 0);
        queryLocal[2] = support[2] - margin;
        shape.coreSupport(1, 0, 0, support, 0);
        queryLocal[3] = support[0] + margin;
        shape.coreSupport(0, 1, 0, support, 0);
        queryLocal[4] = support[1] + margin;
        shape.coreSupport(0, 0, 1, support, 0);
        queryLocal[5] = support[2] + margin;
        CollisionWorld.worldBounds(queryLocal, 0, queryPose, 0, queryBounds);
        double[] b = queryBounds;
        world.staticTree.queryAabb(b[0], b[1], b[2], b[3], b[4], b[5], staticOverlap);
        if (!stopped) {
            world.dynamicTree.queryAabb(b[0], b[1], b[2], b[3], b[4], b[5], dynamicOverlap);
        }
        return found;
    }

    private boolean staticOverlap(int slot) {
        int t = world.staticTriangles[slot];
        return t < 0 ? overlapCandidate(world.staticColliders[slot]) : overlapTriangle(world.staticColliders[slot], t);
    }

    /** The query shape against one triangle of a baked collider; reports the collider at its first touching triangle. */
    private boolean overlapTriangle(int collider, int t) {
        if (!world.isLive(collider) || (world.groups[collider] & mask) == 0 || seen[collider] == stamp) {
            return true;
        }
        world.worldTriangle(collider, t, triangle.corners, localTriangle);
        if (narrow.queryPlaced(queryShape, queryPose, 0, triangle, IDENTITY_POSE, 0, triangleContact) > 0) {
            return true;
        }
        seen[collider] = stamp;
        found++;
        stopped = !visitor.visit(collider);
        return !stopped;
    }

    @AllowAllocation("the world grew")
    private void growSeen(int capacity) {
        seen = Arrays.copyOf(seen, capacity);
    }

    private boolean dynamicOverlap(int slot) {
        return overlapCandidate(world.dynamicColliders[slot]);
    }

    private boolean overlapCandidate(int collider) {
        if (!world.isLive(collider) || (world.groups[collider] & mask) == 0) {
            return true;
        }
        int i = CollisionWorld.POSE * collider;
        Shape shape = world.shapes[collider];
        boolean touching = shape instanceof ConvexShape convex
            ? narrow.overlapsPlaced(queryShape, queryPose, 0, convex, world.poses, i)
            : narrow.contactsPlaced(queryShape, queryPose, 0, (ConcaveShape) shape, world.poses, i, 0, probe) > 0;
        if (!touching) {
            return true;
        }
        found++;
        stopped = !visitor.visit(collider);
        return !stopped;
    }

    // ---------------------------------------------------------------- broad phase

    /**
     * Broad phase for the dynamic colliders in slots {@code from} (inclusive) to {@code to} (exclusive) of the last
     * {@link CollisionWorld#update()}, up to {@link CollisionWorld#dynamicCount()}: appends their pairs to {@code out}
     * in the order {@link CollisionWorld#findPairs} gives them. Workers can split the slots and concatenate their
     * results in slot order to get exactly the single-threaded pairs.
     */
    public void findPairs(int from, int to, PairBuffer out) {
        Objects.checkFromToIndex(from, to, world.dynamicCount());
        if (seen.length < world.capacity()) {
            growSeen(world.capacity());
        }
        pairTarget = out;
        for (int slot = from; slot < to; slot++) {
            pairSlot = slot;
            pairCollider = world.dynamicColliders[slot];
            stamp++;
            double minX = world.dynamicBounds.get(slot, 0);
            double minY = world.dynamicBounds.get(slot, 1);
            double minZ = world.dynamicBounds.get(slot, 2);
            double maxX = world.dynamicBounds.get(slot, 3);
            double maxY = world.dynamicBounds.get(slot, 4);
            double maxZ = world.dynamicBounds.get(slot, 5);
            world.dynamicTree.queryAabb(minX, minY, minZ, maxX, maxY, maxZ, dynamicPairs);
            world.staticTree.queryAabb(minX, minY, minZ, maxX, maxY, maxZ, staticPairs);
        }
    }

    private boolean dynamicPair(int slot) {
        if (slot > pairSlot) {
            pair(world.dynamicColliders[slot]);
        }
        return true;
    }

    private boolean staticPair(int slot) {
        int collider = world.staticColliders[slot];
        if (world.staticTriangles[slot] >= 0) {
            if (seen[collider] == stamp) {
                return true; // another triangle of the same baked collider
            }
            seen[collider] = stamp;
        }
        pair(collider);
        return true;
    }

    private void pair(int other) {
        if (world.canCollide(pairCollider, other)) {
            pairTarget.add(pairCollider, other);
        }
    }

    // ---------------------------------------------------------------- narrow phase over pairs

    /**
     * Runs the narrow phase for pairs {@code from} (inclusive) to {@code to} (exclusive) of {@code pairs}, writing
     * each pair's contacts within {@code maxDistance} to its slots in {@code out}, which must be
     * {@link ContactManifolds#prepare prepared} for all pairs. Contacts point from the pair's first collider to its
     * second. Workers with their own queries may fill disjoint ranges of the same {@code out} concurrently.
     */
    public void collide(PairBuffer pairs, int from, int to, double maxDistance, ContactManifolds out) {
        Objects.checkFromToIndex(from, to, Math.min(pairs.size(), out.size()));
        if (pairContacts.capacity() != out.contactsPerPair()) {
            resizePairContacts(out.contactsPerPair());
        }
        for (int pair = from; pair < to; pair++) {
            collidePair(pair, pairs.first(pair), pairs.second(pair), maxDistance, out);
        }
    }

    @AllowAllocation("when the contacts per pair change")
    private void resizePairContacts(int capacity) {
        pairContacts = new ContactBuffer(capacity);
    }

    private void collidePair(int pair, int a, int b, double maxDistance, ContactManifolds out) {
        if (!world.isLive(a) || !world.isLive(b)) {
            out.setCount(pair, 0); // removed since the pairs were found
            return;
        }
        // The narrow phase wants the convex shape first: swap a moving mesh to B and flip the results back.
        boolean flip = world.isConcave(a);
        int convex = flip ? b : a;
        int other = flip ? a : b;
        double[] poses = world.poses;
        int pc = CollisionWorld.POSE * convex;
        int po = CollisionWorld.POSE * other;
        ConvexShape shape = (ConvexShape) world.shapes[convex];
        if (world.shapes[other] instanceof ConvexShape otherShape) {
            boolean touching = narrow.queryPlaced(shape, poses, pc, otherShape, poses, po, pairContact) <= maxDistance;
            if (touching) {
                out.set(pair, 0, pairContact, false);
            }
            out.setCount(pair, touching ? 1 : 0);
            return;
        }
        int count = narrow.contactsPlaced(shape, poses, pc, (ConcaveShape) world.shapes[other], poses, po, maxDistance,
            pairContacts);
        for (int k = 0; k < count; k++) {
            out.set(pair, k, pairContacts.get(k), flip);
        }
        out.setCount(pair, count);
    }
}
