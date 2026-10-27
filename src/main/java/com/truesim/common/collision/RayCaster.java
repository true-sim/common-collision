package com.truesim.common.collision;

import com.truesim.annotations.AllowAllocation;
import com.truesim.common.math.JitInlining;
import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Intersections;
import com.truesim.common.math.geometry.Ray;
import com.truesim.common.math.spatial.Bvh;
import com.truesim.common.math.spatial.RayHit;

/**
 * Raycasts against any shape placed by a transform: the nearest hit within a maximum distance, with its point, surface
 * normal and, for meshes and heightfields, the triangle and its material.
 *
 * <p>The ray is moved into the shape's unscaled local space, where spheres, capsules and boxes are intersected
 * exactly; scaled shapes therefore work too (a non-uniformly scaled sphere is an ellipsoid). Convex hulls use
 * conservative advancement on the narrow phase's distance query. Meshes descend their own BVH; heightfields walk the
 * cells under the ray. A ray starting inside a convex shape hits at distance 0. Triangles are hit from both sides.
 *
 * <p>All working storage is preallocated: reuse one instance per thread and raycasts allocate nothing. Not
 * thread-safe.
 *
 * <pre>{@code
 * RayCaster caster = new RayCaster();
 * RaycastHit hit = new RaycastHit();
 * if (caster.cast(track, trackPose, wheelCenter, Vec3.DOWN, 1.0, hit)) {
 *     suspension.compress(hit.distance(), hit.normal(), hit.material());
 * }
 * }</pre>
 */
public final class RayCaster {

    static {
        // Inline the thin cast methods into callers so the transforms and rays they create need no allocation.
        JitInlining.include("+com/truesim/common/collision/RayCaster.cast");
    }

    private static final double MISS = Double.POSITIVE_INFINITY;
    private static final double HULL_TOLERANCE = 1e-9;
    private static final int HULL_STEPS = 64;

    private final int hullSteps;

    private final NarrowPhase narrow = new NarrowPhase();
    private final double[] closest = new double[3];
    private final double[] corners = new double[9];
    private final double[] slabOrigin = new double[3];
    private final double[] slabDirection = new double[3];
    private final double[] slabMin = new double[3];
    private final double[] slabMax = new double[3];
    private final RayHit bvhHit = new RayHit();
    private final Bvh.RayTest meshTest;
    private TriangleMeshShape mesh;

    // Result of the local-space cast.
    private double localT;
    private double localNormalX;
    private double localNormalY;
    private double localNormalZ;
    private int localTriangle;

    // Result of the last slab test.
    private double slabEnter;
    private double slabExit;
    private int slabAxis;

    public RayCaster() {
        this(HULL_STEPS);
    }

    /** With an explicit limit on conservative-advancement steps for hulls, e.g. to test reaching it. */
    @AllowAllocation("one ray test per caster")
    RayCaster(int hullSteps) {
        this.hullSteps = hullSteps;
        meshTest = this::meshTriangle;
    }

    /** Casts the ray {@code origin + t * direction}, {@code 0 <= t <= maxDistance}, against the placed shape. */
    public boolean cast(Shape shape, Transform transform, Ray ray, double maxDistance, RaycastHit hit) {
        return cast(shape, transform, ray.origin().x(), ray.origin().y(), ray.origin().z(),
            ray.direction().x(), ray.direction().y(), ray.direction().z(), maxDistance, hit);
    }

    public boolean cast(Shape shape, Transform transform, Vec3 origin, Vec3 direction, double maxDistance, RaycastHit hit) {
        return cast(shape, transform, origin.x(), origin.y(), origin.z(), direction.x(), direction.y(), direction.z(),
            maxDistance, hit);
    }

    public boolean cast(Shape shape, Transform transform, double ox, double oy, double oz, double dx, double dy,
        double dz, double maxDistance, RaycastHit hit) {
        return castPlaced(shape,
            transform.translation().x(), transform.translation().y(), transform.translation().z(),
            transform.rotation().x(), transform.rotation().y(), transform.rotation().z(), transform.rotation().w(),
            transform.scale().x(), transform.scale().y(), transform.scale().z(),
            ox, oy, oz, dx, dy, dz, maxDistance, hit);
    }

    // Kept apart from the thin public methods so those inline into the caller and its transforms need no allocation.
    boolean castPlaced(Shape shape, double tx, double ty, double tz, double qx, double qy, double qz,
        double qw, double sx, double sy, double sz, double ox, double oy, double oz, double dx, double dy, double dz,
        double maxDistance, RaycastHit hit) {
        // Into local space: inverse rotation, then inverse scale. The ray parameter t stays the same.
        double vx = ox - tx;
        double vy = oy - ty;
        double vz = oz - tz;
        double ux = 2 * (-qy * vz + qz * vy);
        double uy = 2 * (-qz * vx + qx * vz);
        double uz = 2 * (-qx * vy + qy * vx);
        double lox = (vx + qw * ux + (-qy * uz + qz * uy)) / sx;
        double loy = (vy + qw * uy + (-qz * ux + qx * uz)) / sy;
        double loz = (vz + qw * uz + (-qx * uy + qy * ux)) / sz;
        ux = 2 * (-qy * dz + qz * dy);
        uy = 2 * (-qz * dx + qx * dz);
        uz = 2 * (-qx * dy + qy * dx);
        double ldx = (dx + qw * ux + (-qy * uz + qz * uy)) / sx;
        double ldy = (dy + qw * uy + (-qz * ux + qx * uz)) / sy;
        double ldz = (dz + qw * uz + (-qx * uy + qy * ux)) / sz;
        hit.collider = RaycastHit.NONE;
        localTriangle = RaycastHit.NONE;
        if (!castLocal(shape, lox, loy, loz, ldx, ldy, ldz, maxDistance)) {
            hit.distance = MISS;
            hit.triangle = RaycastHit.NONE;
            hit.material = RaycastHit.NONE;
            return false;
        }
        double t = localT;
        hit.distance = t;
        hit.pointX = ox + t * dx;
        hit.pointY = oy + t * dy;
        hit.pointZ = oz + t * dz;
        // Normals transform with the inverse transpose: inverse scale, then rotation.
        double nx = localNormalX / sx;
        double ny = localNormalY / sy;
        double nz = localNormalZ / sz;
        double wx = 2 * (qy * nz - qz * ny);
        double wy = 2 * (qz * nx - qx * nz);
        double wz = 2 * (qx * ny - qy * nx);
        double rx = nx + qw * wx + (qy * wz - qz * wy);
        double ry = ny + qw * wy + (qz * wx - qx * wz);
        double rz = nz + qw * wz + (qx * wy - qy * wx);
        double inverse = 1 / Math.sqrt(rx * rx + ry * ry + rz * rz);
        hit.normalX = rx * inverse;
        hit.normalY = ry * inverse;
        hit.normalZ = rz * inverse;
        hit.triangle = localTriangle;
        hit.material = localTriangle == RaycastHit.NONE ? RaycastHit.NONE : ((ConcaveShape) shape).material(localTriangle);
        return true;
    }

    /** Casts in the shape's local space; on a hit sets localT, the (unnormalized) local normal and localTriangle. */
    private boolean castLocal(Shape shape, double ox, double oy, double oz, double dx, double dy, double dz, double max) {
        if (shape instanceof SphereShape sphere) {
            return sphere(sphere.radius(), ox, oy, oz, dx, dy, dz, max);
        } else if (shape instanceof CapsuleShape capsule) {
            return capsule(capsule.halfLength(), capsule.radius(), ox, oy, oz, dx, dy, dz, max);
        } else if (shape instanceof BoxShape box) {
            return box(box.halfExtents().x(), box.halfExtents().y(), box.halfExtents().z(), ox, oy, oz, dx, dy, dz, max);
        } else if (shape instanceof ConvexHullShape hull) {
            return hull(hull, ox, oy, oz, dx, dy, dz, max);
        } else if (shape instanceof TriangleMeshShape triangles) {
            return mesh(triangles, ox, oy, oz, dx, dy, dz, max);
        }
        return heightfield((HeightfieldShape) shape, ox, oy, oz, dx, dy, dz, max);
    }

    private boolean accept(double t, double max, double nx, double ny, double nz) {
        if (t > max) {
            return false;
        }
        localT = t;
        localNormalX = nx;
        localNormalY = ny;
        localNormalZ = nz;
        return true;
    }

    private boolean inside(double dx, double dy, double dz, double max) {
        return accept(0, max, -dx, -dy, -dz);
    }

    // ---------------------------------------------------------------- convex shapes

    private boolean sphere(double r, double ox, double oy, double oz, double dx, double dy, double dz, double max) {
        if (ox * ox + oy * oy + oz * oz <= r * r) {
            return inside(dx, dy, dz, max);
        }
        double t = sphereEntry(0, r, ox, oy, oz, dx, dy, dz);
        return t != MISS && accept(t, max, ox + t * dx, oy + t * dy, oz + t * dz);
    }

    /** First t >= 0 where a ray starting outside enters the sphere at (0, cy, 0); MISS if none. */
    private static double sphereEntry(double cy, double r, double ox, double oy, double oz, double dx, double dy,
        double dz) {
        double my = oy - cy;
        double b = ox * dx + my * dy + oz * dz;
        double a = dx * dx + dy * dy + dz * dz;
        double c = ox * ox + my * my + oz * oz - r * r;
        double discriminant = b * b - a * c;
        if (b >= 0 || discriminant < 0) {
            return MISS; // moving away, or passing by
        }
        return (-b - Math.sqrt(discriminant)) / a;
    }

    private boolean capsule(double h, double r, double ox, double oy, double oz, double dx, double dy, double dz,
        double max) {
        double cy = Math.max(-h, Math.min(h, oy));
        if (ox * ox + (oy - cy) * (oy - cy) + oz * oz <= r * r) {
            return inside(dx, dy, dz, max);
        }
        // The capsule is the union of a cylinder and two spheres: the first entry into any of them.
        double best = MISS;
        double a = dx * dx + dz * dz;
        double b = ox * dx + oz * dz;
        double discriminant = b * b - a * (ox * ox + oz * oz - r * r);
        if (a > 0 && discriminant >= 0) {
            double t = (-b - Math.sqrt(discriminant)) / a;
            if (t >= 0 && Math.abs(oy + t * dy) <= h) {
                best = t;
            }
        }
        double top = sphereEntry(h, r, ox, oy, oz, dx, dy, dz);
        double bottom = sphereEntry(-h, r, ox, oy, oz, dx, dy, dz);
        double t = Math.min(best, Math.min(top, bottom));
        if (t == MISS) {
            return false;
        }
        double y = oy + t * dy;
        // Normal from the nearest point on the core segment.
        return accept(t, max, ox + t * dx, y - Math.max(-h, Math.min(h, y)), oz + t * dz);
    }

    private boolean box(double hx, double hy, double hz, double ox, double oy, double oz, double dx, double dy,
        double dz, double max) {
        if (Math.abs(ox) <= hx && Math.abs(oy) <= hy && Math.abs(oz) <= hz) {
            return inside(dx, dy, dz, max);
        }
        if (!slab(-hx, -hy, -hz, hx, hy, hz, ox, oy, oz, dx, dy, dz)) {
            return false; // starting outside, so a hit enters at slabEnter >= 0
        }
        double sign = slabDirection[slabAxis] > 0 ? -1 : 1;
        return accept(slabEnter, max, slabAxis == 0 ? sign : 0, slabAxis == 1 ? sign : 0, slabAxis == 2 ? sign : 0);
    }

    /**
     * Slab test of the ray against a box: whether the ray's line passes through it, with the entry and exit parameters
     * in slabEnter and slabExit and the axis of the entry face in slabAxis.
     */
    private boolean slab(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, double ox,
        double oy, double oz, double dx, double dy, double dz) {
        slabOrigin[0] = ox;
        slabOrigin[1] = oy;
        slabOrigin[2] = oz;
        slabDirection[0] = dx;
        slabDirection[1] = dy;
        slabDirection[2] = dz;
        slabMin[0] = minX;
        slabMin[1] = minY;
        slabMin[2] = minZ;
        slabMax[0] = maxX;
        slabMax[1] = maxY;
        slabMax[2] = maxZ;
        slabEnter = Double.NEGATIVE_INFINITY;
        slabExit = Double.POSITIVE_INFINITY;
        slabAxis = 0;
        for (int i = 0; i < 3; i++) {
            double d = slabDirection[i];
            double o = slabOrigin[i];
            if (d == 0) {
                if (o < slabMin[i] || o > slabMax[i]) {
                    return false; // parallel and outside this slab
                }
            } else {
                double near = ((d > 0 ? slabMin[i] : slabMax[i]) - o) / d;
                double far = ((d > 0 ? slabMax[i] : slabMin[i]) - o) / d;
                if (near > slabEnter) {
                    slabEnter = near;
                    slabAxis = i;
                }
                slabExit = Math.min(slabExit, far);
            }
        }
        return slabEnter <= slabExit && slabExit >= 0;
    }

    /** Conservative advancement: step along the ray by the distance to the hull, which can never overshoot it. */
    private boolean hull(ConvexHullShape hull, double ox, double oy, double oz, double dx, double dy, double dz,
        double max) {
        double t = 0;
        double nx = -dx;
        double ny = -dy;
        double nz = -dz;
        for (int step = 0; step < hullSteps; step++) {
            double px = ox + t * dx;
            double py = oy + t * dy;
            double pz = oz + t * dz;
            double distance = narrow.pointDistance(hull, px, py, pz, closest);
            if (distance <= HULL_TOLERANCE) {
                return accept(t, max, nx, ny, nz);
            }
            nx = (px - closest[0]) / distance;
            ny = (py - closest[1]) / distance;
            nz = (pz - closest[2]) / distance;
            double approach = -(nx * dx + ny * dy + nz * dz);
            if (approach <= 0) {
                return false; // moving away from the hull
            }
            t += distance / approach;
            if (t > max) {
                return false;
            }
        }
        return false;
    }

    // ---------------------------------------------------------------- triangles

    private boolean mesh(TriangleMeshShape shape, double ox, double oy, double oz, double dx, double dy, double dz,
        double max) {
        mesh = shape;
        if (!shape.raycast(ox, oy, oz, dx, dy, dz, max, meshTest, bvhHit)) {
            return false;
        }
        return triangleHit(shape, bvhHit.primitive(), bvhHit.distance(), dx, dy, dz);
    }

    private double meshTriangle(int triangle, double ox, double oy, double oz, double dx, double dy, double dz,
        double max) {
        return triangle(mesh, triangle, ox, oy, oz, dx, dy, dz);
    }

    private double triangle(ConcaveShape shape, int triangle, double ox, double oy, double oz, double dx, double dy,
        double dz) {
        shape.triangle(triangle, corners, 0);
        return Intersections.rayTriangle(ox, oy, oz, dx, dy, dz,
            corners[0], corners[1], corners[2], corners[3], corners[4], corners[5], corners[6], corners[7], corners[8]);
    }

    /** Accepts a triangle hit with the triangle's normal turned to face the ray. */
    private boolean triangleHit(ConcaveShape shape, int triangle, double t, double dx, double dy, double dz) {
        shape.triangle(triangle, corners, 0);
        double e1x = corners[3] - corners[0];
        double e1y = corners[4] - corners[1];
        double e1z = corners[5] - corners[2];
        double e2x = corners[6] - corners[0];
        double e2y = corners[7] - corners[1];
        double e2z = corners[8] - corners[2];
        double nx = e1y * e2z - e1z * e2y;
        double ny = e1z * e2x - e1x * e2z;
        double nz = e1x * e2y - e1y * e2x;
        double facing = nx * dx + ny * dy + nz * dz > 0 ? -1 : 1;
        localTriangle = triangle;
        localT = t;
        localNormalX = nx * facing;
        localNormalY = ny * facing;
        localNormalZ = nz * facing;
        return true;
    }

    /** Walks the cells under the ray front to back (a 2D DDA over x and z); the first cell with a hit has the nearest. */
    private boolean heightfield(HeightfieldShape field, double ox, double oy, double oz, double dx, double dy,
        double dz, double max) {
        double sx = field.cellSizeX();
        double sz = field.cellSizeZ();
        int cellsX = field.cellsX();
        int cellsZ = field.cellsZ();
        double minY = field.localBounds().min().y();
        double maxY = field.localBounds().max().y();
        if (!slab(0, minY, 0, cellsX * sx, maxY, cellsZ * sz, ox, oy, oz, dx, dy, dz)) {
            return false;
        }
        double t = Math.max(0, slabEnter);
        double end = Math.min(slabExit, max);
        if (t > end) {
            return false;
        }
        int x = Math.max(0, Math.min(cellsX - 1, (int) Math.floor((ox + t * dx) / sx)));
        int z = Math.max(0, Math.min(cellsZ - 1, (int) Math.floor((oz + t * dz) / sz)));
        int stepX = dx > 0 ? 1 : -1;
        int stepZ = dz > 0 ? 1 : -1;
        double nextX = dx == 0 ? MISS : ((x + (dx > 0 ? 1 : 0)) * sx - ox) / dx;
        double nextZ = dz == 0 ? MISS : ((z + (dz > 0 ? 1 : 0)) * sz - oz) / dz;
        double deltaX = sx / Math.abs(dx);
        double deltaZ = sz / Math.abs(dz);
        while (true) {
            double exit = Math.min(end, Math.min(nextX, nextZ));
            double y0 = oy + t * dy;
            double y1 = oy + exit * dy;
            double h00 = field.height(x, z);
            double h10 = field.height(x + 1, z);
            double h01 = field.height(x, z + 1);
            double h11 = field.height(x + 1, z + 1);
            // Skip cells the ray passes entirely above or below.
            if (Math.max(y0, y1) >= Math.min(Math.min(h00, h10), Math.min(h01, h11))
                && Math.min(y0, y1) <= Math.max(Math.max(h00, h10), Math.max(h01, h11))) {
                int first = 2 * (z * cellsX + x);
                double t0 = triangle(field, first, ox, oy, oz, dx, dy, dz);
                double t1 = triangle(field, first + 1, ox, oy, oz, dx, dy, dz);
                double hit = Math.min(t0, t1);
                if (hit != MISS && hit <= max) {
                    return triangleHit(field, t0 <= t1 ? first : first + 1, hit, dx, dy, dz);
                }
            }
            if (exit >= end) {
                return false;
            }
            if (nextX < nextZ) {
                x += stepX;
                t = nextX;
                nextX += deltaX;
            } else {
                z += stepZ;
                t = nextZ;
                nextZ += deltaZ;
            }
            if (x < 0 || x >= cellsX || z < 0 || z >= cellsZ) {
                return false;
            }
        }
    }
}
