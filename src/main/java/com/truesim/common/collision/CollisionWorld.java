package com.truesim.common.collision;

import com.truesim.annotations.AllowAllocation;
import com.truesim.common.math.Quat;
import com.truesim.common.math.Transform;
import com.truesim.common.math.TransformArray;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;
import com.truesim.common.math.geometry.AabbArray;
import com.truesim.common.math.spatial.Bvh;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.util.Arrays;
import java.util.Objects;

/**
 * A set of colliders, each a {@link Shape} placed by a pose, with two bounding volume hierarchies for fast queries: one
 * over static colliders (rebuilt only when they change) and one over dynamic colliders (refit every
 * {@link #update()}, rebuilt when it has degraded).
 *
 * <p>The world is separate from any scene: colliders are ints, and poses are either set directly or copied from a
 * shared {@link TransformArray} in an explicit {@link #syncPoses} step. A typical step:
 *
 * <pre>{@code
 * world.syncPoses(bodies);     // or setPose(...) per collider
 * world.update();              // bounds and trees
 * world.findPairs(pairs);      // broad phase: candidate pairs
 * query.raycast(...);          // one CollisionQuery per thread
 * }</pre>
 *
 * <p>Each collider has a material id and user data (free for the owner, e.g. a body index), and a collision filter:
 * two colliders can collide when each one's group shares a bit with the other's mask. An id carries a generation:
 * once its collider is removed it names nothing, even after a new collider takes its slot (until the slot was reused
 * 128 times), so a stale id is caught instead of naming another collider. Changing the world is not thread-safe; after {@link #update()}, any number of threads can query it through
 * their own {@link CollisionQuery}.
 */
public final class CollisionWorld {

    /** Doubles per pose: translation, rotation (x, y, z, w), scale. */
    static final int POSE = 10;
    private static final int USED = 1;
    private static final int STATIC = 2;
    private static final int CONCAVE = 4;
    private static final int BAKED = 8;
    /** Bits of an id holding the collider's index; the bits above hold the generation of its slot. */
    static final int INDEX_BITS = 24;
    private static final int INDEX = (1 << INDEX_BITS) - 1;
    private static final int GENERATIONS = 1 << (31 - INDEX_BITS);

    Shape[] shapes = new Shape[0];
    double[] poses = new double[0];
    int[] groups = new int[0];
    private double[] localBounds = new double[0];
    private int[] flags = new int[0];
    private int[] masks = new int[0];
    private int[] materials = new int[0];
    private int[] userData = new int[0];
    private int[] sources = new int[0];
    private int[] generations = new int[0]; // per slot, bumped as its collider goes
    int maxColliders = INDEX + 1; // as many as an id has index bits for; fewer in tests
    private int[] free = new int[0];
    private int freeCount;
    private int highWater;
    private int count;

    // Broad phase.
    AabbArray staticBounds = AabbArray.onHeap(0);
    AabbArray dynamicBounds = AabbArray.onHeap(0);
    Bvh staticTree = Bvh.build(staticBounds);
    Bvh dynamicTree = Bvh.build(dynamicBounds);
    int[] staticColliders = new int[0];
    /** For each static tree entry, the triangle of a baked collider, or -1 for a whole collider. */
    int[] staticTriangles = new int[0];
    int[] dynamicColliders = new int[0];
    private boolean structureChanged;
    private boolean staticMoved;
    private double rebuildThreshold = 1.5;
    private double boundsMargin;

    // Runs findPairs on the calling thread; workers use their own queries.
    private final CollisionQuery pairQuery = new CollisionQuery(this);

    // ---------------------------------------------------------------- colliders

    /** Adds a collider that never moves (or rarely: moving it rebuilds the static tree). Returns its id. */
    public int addStatic(Shape shape, Transform pose) {
        return add(shape, pose, true, false);
    }

    /**
     * Adds a static collider; with {@code bake}, a mesh or heightfield goes into the static tree triangle by triangle,
     * in world space, instead of as one entry with its own tree. Raycasts and overlap queries then descend one tree
     * instead of two, which pays off for large static geometry such as a track; the cost is a static tree rebuild
     * whenever baked geometry is added, removed or moved, and a copy of its triangles' bounds per collider. Results
     * are the same either way.
     */
    public int addStatic(Shape shape, Transform pose, boolean bake) {
        if (bake && !(shape instanceof ConcaveShape)) {
            throw new IllegalArgumentException("Only meshes and heightfields can be baked: " + shape);
        }
        return add(shape, pose, true, bake);
    }

    /** Adds a collider that moves. Returns its id. */
    public int addDynamic(Shape shape, Transform pose) {
        return add(shape, pose, false, false);
    }

    @AllowAllocation("adding colliders")
    private int add(Shape shape, Transform pose, boolean isStatic, boolean bake) {
        checkPose(pose);
        int id;
        if (freeCount > 0) {
            id = free[--freeCount];
        } else {
            if (highWater == maxColliders) {
                throw new IllegalStateException("At most " + maxColliders + " colliders");
            }
            id = highWater++;
            if (id == flags.length) {
                grow(Math.max(16, 2 * id));
            }
        }
        shapes[id] = shape;
        flags[id] = USED | (isStatic ? STATIC : 0) | (shape instanceof ConcaveShape ? CONCAVE : 0) | (bake ? BAKED : 0);
        Aabb bounds = shape.localBounds();
        localBounds[6 * id] = bounds.min().x();
        localBounds[6 * id + 1] = bounds.min().y();
        localBounds[6 * id + 2] = bounds.min().z();
        localBounds[6 * id + 3] = bounds.max().x();
        localBounds[6 * id + 4] = bounds.max().y();
        localBounds[6 * id + 5] = bounds.max().z();
        groups[id] = 1;
        masks[id] = -1;
        materials[id] = 0;
        userData[id] = 0;
        sources[id] = -1;
        count++;
        structureChanged = true;
        int handle = id(id);
        setPose(handle, pose);
        return handle;
    }

    @AllowAllocation("adding colliders")
    private void grow(int capacity) {
        shapes = Arrays.copyOf(shapes, capacity);
        poses = Arrays.copyOf(poses, POSE * capacity);
        localBounds = Arrays.copyOf(localBounds, 6 * capacity);
        flags = Arrays.copyOf(flags, capacity);
        groups = Arrays.copyOf(groups, capacity);
        masks = Arrays.copyOf(masks, capacity);
        materials = Arrays.copyOf(materials, capacity);
        userData = Arrays.copyOf(userData, capacity);
        sources = Arrays.copyOf(sources, capacity);
        generations = Arrays.copyOf(generations, capacity);
        free = Arrays.copyOf(free, capacity);
    }

    /** Number of collider ids in use or freed so far: every id is below it. */
    int capacity() {
        return highWater;
    }

    /** Removes the collider: its id names nothing from now on; its slot goes to a later add, under a new id. */
    public void remove(int collider) {
        collider = slot(collider);
        generations[collider] = (generations[collider] + 1) % GENERATIONS; // the old id names nothing now
        flags[collider] = 0;
        shapes[collider] = null;
        free[freeCount++] = collider;
        count--;
        structureChanged = true;
    }

    /** Whether {@code collider} is a live collider's id: not removed, and not an id of a slot reused since. */
    public boolean contains(int collider) {
        int index = collider & INDEX;
        return collider >= 0 && index < highWater && (flags[index] & USED) != 0
            && generations[index] == collider >>> INDEX_BITS;
    }

    /** Number of colliders. */
    public int size() {
        return count;
    }

    public Shape shape(int collider) {
        collider = slot(collider);
        return shapes[collider];
    }

    /** Whether the collider was added baked: its triangles are entries of the static tree. */
    public boolean isBaked(int collider) {
        return baked(slot(collider));
    }

    private boolean baked(int index) {
        return (flags[index] & BAKED) != 0;
    }

    public boolean isStatic(int collider) {
        collider = slot(collider);
        return (flags[collider] & STATIC) != 0;
    }

    public Transform pose(int collider) {
        collider = slot(collider);
        int p = POSE * collider;
        return Transform.of(Vec3.of(poses[p], poses[p + 1], poses[p + 2]),
            new Quat(poses[p + 3], poses[p + 4], poses[p + 5], poses[p + 6]),
            Vec3.of(poses[p + 7], poses[p + 8], poses[p + 9]));
    }

    /**
     * Moves the collider; takes effect for queries at the next {@link #update()}.
     *
     * @throws IllegalArgumentException if the pose is not finite, or scales to nothing on an axis
     */
    public void setPose(int collider, Transform pose) {
        collider = slot(collider);
        checkPose(pose);
        int p = POSE * collider;
        poses[p] = pose.translation().x();
        poses[p + 1] = pose.translation().y();
        poses[p + 2] = pose.translation().z();
        poses[p + 3] = pose.rotation().x();
        poses[p + 4] = pose.rotation().y();
        poses[p + 5] = pose.rotation().z();
        poses[p + 6] = pose.rotation().w();
        poses[p + 7] = pose.scale().x();
        poses[p + 8] = pose.scale().y();
        poses[p + 9] = pose.scale().z();
        if ((flags[collider] & STATIC) != 0) {
            staticMoved = true;
        }
    }

    private static void checkPose(Transform pose) {
        Vec3 t = pose.translation();
        Quat r = pose.rotation();
        Vec3 s = pose.scale();
        if (!Double.isFinite(t.x() + t.y() + t.z() + r.x() + r.y() + r.z() + r.w())
            || !Double.isFinite(s.x() + s.y() + s.z()) || s.x() == 0 || s.y() == 0 || s.z() == 0) {
            throw new IllegalArgumentException("Not a finite pose with a scale on every axis: " + pose);
        }
    }

    /**
     * Makes {@link #syncPoses} copy this collider's pose from index {@code index} of the source array; -1 stops it.
     */
    public void setPoseSource(int collider, int index) {
        collider = slot(collider);
        sources[collider] = Math.max(-1, index);
    }

    public int poseSource(int collider) {
        collider = slot(collider);
        return sources[collider];
    }

    /**
     * Copies the pose of every collider with a pose source from {@code source}, e.g. the simulation's bodies. Static
     * colliders whose pose did not change leave the static tree as it is.
     */
    public void syncPoses(TransformArray source) {
        MemorySegment from = source.segment();
        for (int collider = 0; collider < highWater; collider++) {
            if ((flags[collider] & USED) != 0 && sources[collider] >= 0) {
                // A TransformArray stores the same ten doubles in the same order as the world's poses.
                long at = Objects.checkIndex(sources[collider], source.size()) * source.stride();
                if ((flags[collider] & STATIC) != 0) {
                    if (samePose(from, at, collider)) {
                        continue; // a rebuild of the static tree for nothing
                    }
                    staticMoved = true;
                }
                MemorySegment.copy(from, ValueLayout.JAVA_DOUBLE_UNALIGNED, at, poses, POSE * collider, POSE);
            }
        }
    }

    private boolean samePose(MemorySegment from, long at, int collider) {
        for (int i = 0; i < POSE; i++) {
            if (from.get(ValueLayout.JAVA_DOUBLE_UNALIGNED, at + 8L * i) != poses[POSE * collider + i]) {
                return false;
            }
        }
        return true;
    }

    public int material(int collider) {
        collider = slot(collider);
        return materials[collider];
    }

    public void setMaterial(int collider, int material) {
        collider = slot(collider);
        materials[collider] = material;
    }

    public int userData(int collider) {
        collider = slot(collider);
        return userData[collider];
    }

    public void setUserData(int collider, int data) {
        collider = slot(collider);
        userData[collider] = data;
    }

    public int group(int collider) {
        collider = slot(collider);
        return groups[collider];
    }

    public int mask(int collider) {
        collider = slot(collider);
        return masks[collider];
    }

    /**
     * Sets the collision filter: two colliders can collide when {@code (groupA & maskB) != 0} and
     * {@code (groupB & maskA) != 0}; queries find colliders whose group shares a bit with the query's mask. By default
     * the group is 1 and the mask is all bits.
     */
    public void setFilter(int collider, int group, int mask) {
        collider = slot(collider);
        groups[collider] = group;
        masks[collider] = mask;
    }

    /** The index of collider {@code id}: where its data is kept. */
    private int slot(int id) {
        if (!contains(id)) {
            throw new IllegalArgumentException("No collider " + id + (id >= 0 && (id & INDEX) < highWater
                && (flags[id & INDEX] & USED) != 0 ? " (removed: its slot holds a newer one)" : ""));
        }
        return id & INDEX;
    }

    /** The id of the collider at {@code index}: the index, and the slot's generation above it. */
    int id(int index) {
        return index | generations[index] << INDEX_BITS;
    }

    /** The index of a live collider's {@code id}, or -1 for a removed one (removed since a query found it, say). */
    int indexOf(int id) {
        return contains(id) ? id & INDEX : -1;
    }

    boolean isLive(int collider) {
        return (flags[collider] & USED) != 0;
    }

    boolean isConcave(int collider) {
        return (flags[collider] & CONCAVE) != 0;
    }

    // ---------------------------------------------------------------- update

    /** When the dynamic tree is this much slower than after a build ({@link Bvh#degradation()}), it is rebuilt. */
    public double rebuildThreshold() {
        return rebuildThreshold;
    }

    public void setRebuildThreshold(double threshold) {
        rebuildThreshold = threshold;
    }

    public double boundsMargin() {
        return boundsMargin;
    }

    /**
     * Grows every collider's bounds by {@code margin}, from the next {@link #update()}. Set it to at least the contact
     * distance passed to {@link CollisionQuery#collide}, so pairs closer than that are found even when their exact
     * bounds do not overlap yet. Default 0.
     */
    public void setBoundsMargin(double margin) {
        if (!(margin >= 0)) {
            throw new IllegalArgumentException("Margin must not be negative: " + margin);
        }
        boundsMargin = margin;
        structureChanged = true;
    }

    /**
     * Brings bounds and trees up to date with the colliders' poses: refits the dynamic tree (rebuilding it once it has
     * degraded), rebuilds the static tree if a static collider moved, and rebuilds both after colliders were added or
     * removed. Allocation-free unless a tree is rebuilt: after colliders were added or removed, a static one moved, or
     * the dynamic tree degraded past the threshold.
     */
    public void update() {
        if (structureChanged) {
            rebuild();
            return;
        }
        if (staticMoved) {
            for (int slot = 0; slot < staticColliders.length; slot++) {
                writeStaticBounds(slot);
            }
            staticTree.rebuild();
            staticMoved = false;
        }
        for (int slot = 0; slot < dynamicColliders.length; slot++) {
            writeBounds(dynamicColliders[slot], dynamicBounds, slot);
        }
        dynamicTree.refit();
        if (dynamicTree.degradation() > rebuildThreshold) {
            dynamicTree.rebuild();
        }
    }

    @AllowAllocation("colliders were added or removed")
    private void rebuild() {
        // Static tree entries: one per static collider, or one per triangle of a baked one.
        int statics = 0;
        int staticEntries = 0;
        for (int collider = 0; collider < highWater; collider++) {
            if ((flags[collider] & (USED | STATIC)) == (USED | STATIC)) {
                statics++;
                staticEntries += baked(collider) ? ((ConcaveShape) shapes[collider]).triangleCount() : 1;
            }
        }
        staticColliders = new int[staticEntries];
        staticTriangles = new int[staticEntries];
        dynamicColliders = new int[count - statics];
        int s = 0;
        int d = 0;
        for (int collider = 0; collider < highWater; collider++) {
            if ((flags[collider] & (USED | STATIC)) == (USED | STATIC)) {
                int triangles = baked(collider) ? ((ConcaveShape) shapes[collider]).triangleCount() : 0;
                for (int t = 0; t < Math.max(1, triangles); t++) {
                    staticColliders[s] = collider;
                    staticTriangles[s++] = triangles == 0 ? -1 : t;
                }
            } else if ((flags[collider] & USED) != 0) {
                dynamicColliders[d++] = collider;
            }
        }
        staticBounds = AabbArray.onHeap(staticColliders.length);
        dynamicBounds = AabbArray.onHeap(dynamicColliders.length);
        for (int slot = 0; slot < staticColliders.length; slot++) {
            writeStaticBounds(slot);
        }
        for (int slot = 0; slot < dynamicColliders.length; slot++) {
            writeBounds(dynamicColliders[slot], dynamicBounds, slot);
        }
        staticTree = Bvh.build(staticBounds);
        dynamicTree = Bvh.build(dynamicBounds);
        structureChanged = false;
        staticMoved = false;
    }

    /** Bounds of a static tree entry: the whole collider, or one triangle of a baked one. */
    private void writeStaticBounds(int slot) {
        int triangle = staticTriangles[slot];
        if (triangle < 0) {
            writeBounds(staticColliders[slot], staticBounds, slot);
            return;
        }
        double[] c = triangleScratch;
        worldTriangle(staticColliders[slot], triangle, c, localTriangleScratch);
        double m = boundsMargin;
        staticBounds.set(slot,
            Math.min(c[0], Math.min(c[3], c[6])) - m, Math.min(c[1], Math.min(c[4], c[7])) - m,
            Math.min(c[2], Math.min(c[5], c[8])) - m, Math.max(c[0], Math.max(c[3], c[6])) + m,
            Math.max(c[1], Math.max(c[4], c[7])) + m, Math.max(c[2], Math.max(c[5], c[8])) + m);
    }

    private final double[] triangleScratch = new double[9];
    private final double[] localTriangleScratch = new double[9];

    /**
     * World-space corners of {@code triangle} of the concave collider, written to {@code out[0..8]}; {@code local} is
     * scratch space for nine doubles.
     */
    void worldTriangle(int collider, int triangle, double[] out, double[] local) {
        ((ConcaveShape) shapes[collider]).triangle(triangle, local, 0);
        int p = POSE * collider;
        double qx = poses[p + 3];
        double qy = poses[p + 4];
        double qz = poses[p + 5];
        double qw = poses[p + 6];
        for (int i = 0; i < 9; i += 3) {
            double px = local[i] * poses[p + 7];
            double py = local[i + 1] * poses[p + 8];
            double pz = local[i + 2] * poses[p + 9];
            double wx = 2 * (qy * pz - qz * py);
            double wy = 2 * (qz * px - qx * pz);
            double wz = 2 * (qx * py - qy * px);
            out[i] = poses[p] + px + qw * wx + (qy * wz - qz * wy);
            out[i + 1] = poses[p + 1] + py + qw * wy + (qz * wx - qx * wz);
            out[i + 2] = poses[p + 2] + pz + qw * wz + (qx * wy - qy * wx);
        }
    }

    private void writeBounds(int collider, AabbArray target, int slot) {
        double[] box = boundsScratch;
        worldBounds(localBounds, 6 * collider, poses, POSE * collider, box);
        double m = boundsMargin;
        target.set(slot, box[0] - m, box[1] - m, box[2] - m, box[3] + m, box[4] + m, box[5] + m);
    }

    private final double[] boundsScratch = new double[6];

    /** The collider's world bounds, computed from its current pose. */
    public Aabb bounds(int collider) {
        collider = slot(collider);
        double[] box = new double[6];
        worldBounds(localBounds, 6 * collider, poses, POSE * collider, box);
        return new Aabb(Vec3.of(box[0], box[1], box[2]), Vec3.of(box[3], box[4], box[5]));
    }

    /**
     * World bounds of the local box {@code local[li..li + 5]} (min, max) placed by the pose at {@code pose[pi]}: the box
     * around the transformed box, exact for its corners. Written to {@code out[0..5]}.
     */
    static void worldBounds(double[] local, int li, double[] pose, int pi, double[] out) {
        double cx = (local[li] + local[li + 3]) / 2;
        double cy = (local[li + 1] + local[li + 4]) / 2;
        double cz = (local[li + 2] + local[li + 5]) / 2;
        double ex = (local[li + 3] - local[li]) / 2;
        double ey = (local[li + 4] - local[li + 1]) / 2;
        double ez = (local[li + 5] - local[li + 2]) / 2;
        double qx = pose[pi + 3];
        double qy = pose[pi + 4];
        double qz = pose[pi + 5];
        double qw = pose[pi + 6];
        double sx = pose[pi + 7];
        double sy = pose[pi + 8];
        double sz = pose[pi + 9];
        // Rotation times scale, column by column.
        double m00 = (1 - 2 * (qy * qy + qz * qz)) * sx;
        double m01 = 2 * (qx * qy - qz * qw) * sy;
        double m02 = 2 * (qx * qz + qy * qw) * sz;
        double m10 = 2 * (qx * qy + qz * qw) * sx;
        double m11 = (1 - 2 * (qx * qx + qz * qz)) * sy;
        double m12 = 2 * (qy * qz - qx * qw) * sz;
        double m20 = 2 * (qx * qz - qy * qw) * sx;
        double m21 = 2 * (qy * qz + qx * qw) * sy;
        double m22 = (1 - 2 * (qx * qx + qy * qy)) * sz;
        double wx = pose[pi] + m00 * cx + m01 * cy + m02 * cz;
        double wy = pose[pi + 1] + m10 * cx + m11 * cy + m12 * cz;
        double wz = pose[pi + 2] + m20 * cx + m21 * cy + m22 * cz;
        double rx = Math.abs(m00) * ex + Math.abs(m01) * ey + Math.abs(m02) * ez;
        double ry = Math.abs(m10) * ex + Math.abs(m11) * ey + Math.abs(m12) * ez;
        double rz = Math.abs(m20) * ex + Math.abs(m21) * ey + Math.abs(m22) * ez;
        out[0] = wx - rx;
        out[1] = wy - ry;
        out[2] = wz - rz;
        out[3] = wx + rx;
        out[4] = wy + ry;
        out[5] = wz + rz;
    }

    // ---------------------------------------------------------------- queries

    /** A query context for one thread. */
    public CollisionQuery newQuery() {
        return new CollisionQuery(this);
    }

    /**
     * Broad phase: the pairs of colliders whose bounds overlap and whose filters let them collide, as of the last
     * {@link #update()}. Each pair has at least one dynamic collider (the first); pairs of two meshes or heightfields
     * are left out. The order is deterministic.
     */
    public void findPairs(PairBuffer out) {
        out.clear();
        pairQuery.findPairs(0, dynamicColliders.length, out);
    }

    /** Number of dynamic colliders as of the last {@link #update()}: the range {@link CollisionQuery#findPairs} splits. */
    public int dynamicCount() {
        return dynamicColliders.length;
    }

    /** Whether a pair of live colliders may collide: filters match and not both are meshes or heightfields. */
    boolean canCollide(int a, int b) {
        // Colliders removed since the last update may still be in the trees.
        return isLive(a) && isLive(b) && (groups[a] & masks[b]) != 0 && (groups[b] & masks[a]) != 0
            && !(isConcave(a) && isConcave(b));
    }
}
