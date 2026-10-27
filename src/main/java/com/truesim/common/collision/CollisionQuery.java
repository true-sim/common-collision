package com.truesim.common.collision;

import com.truesim.annotations.AllowAllocation;
import com.truesim.common.math.JitInlining;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.spatial.Bvh;
import com.truesim.common.math.spatial.RayHit;

/**
 * Queries against a {@link CollisionWorld} from one thread: raycasts and overlaps. Holds the working storage, so
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
    private final double[] queryPose = new double[CollisionWorld.POSE];
    private final double[] queryLocal = new double[6];
    private final double[] queryBounds = new double[6];
    private final double[] support = new double[3];
    private final Bvh.RayTest staticRay;
    private final Bvh.RayTest dynamicRay;
    private final Bvh.Visitor staticOverlap;
    private final Bvh.Visitor dynamicOverlap;
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
        return ray(world.staticColliders[slot], ox, oy, oz, dx, dy, dz, max);
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
        return overlapCandidate(world.staticColliders[slot]);
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
}
