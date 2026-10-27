package com.truesim.common.collision;

import com.truesim.common.math.JitInlining;
import com.truesim.common.math.Transform;

/**
 * Narrow-phase queries between two convex shapes: overlap, signed distance, closest points and penetration.
 *
 * <p>GJK finds the distance between the shapes' cores (points, segments, boxes, hulls); round shapes add their
 * margin on top, which is exact. When the cores overlap, EPA finds the core penetration and the margins are added
 * again. All working storage is preallocated: reuse one instance per thread and queries allocate nothing. Not
 * thread-safe.
 *
 * <pre>{@code
 * NarrowPhase narrow = new NarrowPhase();
 * Contact contact = new Contact();
 * if (narrow.query(carShape, carPose, wallShape, wallPose, contact) < 0) {
 *     push(car, contact.normal(), contact.depth());
 * }
 * }</pre>
 */
public final class NarrowPhase {

    static {
        // Inline the thin query methods into callers even after they were compiled on their own, so that the
        // transforms callers create for a query need no allocation.
        JitInlining.include(
            "+com/truesim/common/collision/NarrowPhase.query", "+com/truesim/common/collision/NarrowPhase.overlaps",
            "+com/truesim/common/collision/Pose.set");
    }

    private static final double OVERLAP_EPSILON_SQUARED = 1e-20;
    private static final double CORE_EPSILON = 1e-10;
    private static final double RELATIVE_TOLERANCE = 1e-12;
    private static final double EPA_TOLERANCE = 1e-10;
    private static final double INDEPENDENCE_EPSILON = 1e-10;

    /** Directions tried, in order, to grow a degenerate simplex into a tetrahedron. */
    private static final double[] DIRECTIONS = {
        1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1,
        1, 1, 1, -1, -1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1, 1, 1,
    };

    private final int maxIterations;
    private final int maxVertices;
    private final int maxFaces;

    private final Pose poseA = new Pose();
    private final Pose poseB = new Pose();

    // GJK simplex: Minkowski difference points w = a - b and the shape points a, b they came from.
    private final double[] w = new double[12];
    private final double[] a = new double[12];
    private final double[] b = new double[12];
    private final double[] lambda = new double[4];
    private int count;
    private double vx;
    private double vy;
    private double vz;

    // Best candidate while solving the simplex.
    private int bestMask;
    private double bestDistance;
    private final double[] bestLambda = new double[4];

    // EPA polytope.
    private final double[] ew;
    private final double[] ea;
    private final double[] eb;
    private int vertexCount;
    private final int[] faces;
    private final double[] faceNormals;
    private final double[] faceDistances;
    private final boolean[] faceAlive;
    private int faceCount;
    private final int[] edges;
    private int edgeCount;

    public NarrowPhase() {
        this(64, 128, 256);
    }

    /** With explicit safety limits, e.g. to test what happens when they are reached. */
    NarrowPhase(int maxIterations, int maxVertices, int maxFaces) {
        this.maxIterations = maxIterations;
        this.maxVertices = maxVertices;
        this.maxFaces = maxFaces;
        ew = new double[3 * maxVertices];
        ea = new double[3 * maxVertices];
        eb = new double[3 * maxVertices];
        faces = new int[3 * maxFaces];
        faceNormals = new double[3 * maxFaces];
        faceDistances = new double[maxFaces];
        faceAlive = new boolean[maxFaces];
        edges = new int[2 * 3 * maxFaces];
    }

    // ---------------------------------------------------------------- queries

    /** Whether the shapes touch or overlap. Cheaper than {@link #query}: no closest points, no penetration depth. */
    public boolean overlaps(ConvexShape shapeA, Transform transformA, ConvexShape shapeB, Transform transformB) {
        poseA.set(shapeA, transformA);
        poseB.set(shapeB, transformB);
        return coreDistance() <= poseA.margin + poseB.margin;
    }

    /**
     * Signed distance between the shapes, with normal and surface points written to {@code contact}: positive when
     * apart (closest points), negative when penetrating (deepest points; minus the penetration depth).
     */
    public double query(ConvexShape shapeA, Transform transformA, ConvexShape shapeB, Transform transformB, Contact contact) {
        poseA.set(shapeA, transformA);
        poseB.set(shapeB, transformB);
        return solve(contact);
    }

    // Kept apart from the thin public methods so those inline into the caller and its transforms need no allocation.
    private double solve(Contact contact) {
        double core = coreDistance();
        if (core > CORE_EPSILON) {
            double invCore = 1 / core;
            closestPoints();
            // v = a - b, so the normal from A to B is -v.
            write(contact, -vx * invCore, -vy * invCore, -vz * invCore, core);
            return contact.distance;
        }
        return penetration(contact);
    }

    /** Fills the contact from core points (in the a/b scratch slots 0) and a core signed distance. */
    private void write(Contact contact, double nx, double ny, double nz, double coreDistance) {
        double ma = poseA.margin;
        double mb = poseB.margin;
        contact.normalX = nx;
        contact.normalY = ny;
        contact.normalZ = nz;
        contact.distance = coreDistance - ma - mb;
        contact.ax = pax + nx * ma;
        contact.ay = pay + ny * ma;
        contact.az = paz + nz * ma;
        contact.bx = pbx - nx * mb;
        contact.by = pby - ny * mb;
        contact.bz = pbz - nz * mb;
    }

    private double pax;
    private double pay;
    private double paz;
    private double pbx;
    private double pby;
    private double pbz;

    /** Closest core points from the simplex weights. */
    private void closestPoints() {
        pax = 0;
        pay = 0;
        paz = 0;
        pbx = 0;
        pby = 0;
        pbz = 0;
        for (int i = 0; i < count; i++) {
            double l = lambda[i];
            pax += l * a[3 * i];
            pay += l * a[3 * i + 1];
            paz += l * a[3 * i + 2];
            pbx += l * b[3 * i];
            pby += l * b[3 * i + 1];
            pbz += l * b[3 * i + 2];
        }
    }

    // ---------------------------------------------------------------- GJK

    /** Distance between the cores; 0 if they touch or overlap (the simplex then contains the origin). */
    private double coreDistance() {
        double dx = poseA.centerX - poseB.centerX;
        double dy = poseA.centerY - poseB.centerY;
        double dz = poseA.centerZ - poseB.centerZ;
        boolean same = dx == 0 && dy == 0 && dz == 0;
        count = 0;
        addSupport(same ? 1 : dx, dy, dz);
        lambda[0] = 1;
        vx = w[0];
        vy = w[1];
        vz = w[2];
        for (int iteration = 0; iteration < maxIterations; iteration++) {
            double vv = vx * vx + vy * vy + vz * vz;
            if (vv <= OVERLAP_EPSILON_SQUARED) {
                return 0;
            }
            addSupport(-vx, -vy, -vz);
            int n = 3 * (count - 1);
            double vw = vx * w[n] + vy * w[n + 1] + vz * w[n + 2];
            if (vv - vw <= RELATIVE_TOLERANCE * vv) {
                count--; // no point closer to the origin exists: converged
                return Math.sqrt(vv);
            }
            solve();
            if (count == 4) {
                return 0; // the origin is inside the tetrahedron
            }
        }
        return Math.sqrt(vx * vx + vy * vy + vz * vz);
    }

    /** Adds the Minkowski difference support point in direction d as simplex vertex {@code count}. */
    private void addSupport(double dx, double dy, double dz) {
        int i = 3 * count;
        poseA.coreSupport(dx, dy, dz, a, i);
        poseB.coreSupport(-dx, -dy, -dz, b, i);
        w[i] = a[i] - b[i];
        w[i + 1] = a[i + 1] - b[i + 1];
        w[i + 2] = a[i + 2] - b[i + 2];
        count++;
    }

    /** Reduces the simplex to the feature closest to the origin and updates v and the weights. */
    private void solve() {
        bestDistance = Double.POSITIVE_INFINITY;
        if (count == 2) {
            segment(0, 1);
        } else if (count == 3) {
            triangle(0, 1, 2);
        } else {
            tetrahedron();
        }
        int kept = 0;
        vx = 0;
        vy = 0;
        vz = 0;
        for (int i = 0; i < count; i++) {
            if ((bestMask & (1 << i)) != 0) {
                double l = bestLambda[i];
                System.arraycopy(w, 3 * i, w, 3 * kept, 3);
                System.arraycopy(a, 3 * i, a, 3 * kept, 3);
                System.arraycopy(b, 3 * i, b, 3 * kept, 3);
                lambda[kept] = l;
                vx += l * w[3 * kept];
                vy += l * w[3 * kept + 1];
                vz += l * w[3 * kept + 2];
                kept++;
            }
        }
        count = kept;
    }

    private void segment(int i, int j) {
        double ax = w[3 * i];
        double ay = w[3 * i + 1];
        double az = w[3 * i + 2];
        double ex = w[3 * j] - ax;
        double ey = w[3 * j + 1] - ay;
        double ez = w[3 * j + 2] - az;
        double ee = ex * ex + ey * ey + ez * ez;
        double t = ee == 0 ? 0 : Math.max(0, Math.min(1, -(ax * ex + ay * ey + az * ez) / ee));
        offer(i, 1 - t, j, t, -1, 0);
    }

    private void triangle(int i, int j, int k) {
        int pi = 3 * i;
        int pj = 3 * j;
        int pk = 3 * k;
        double abx = w[pj] - w[pi];
        double aby = w[pj + 1] - w[pi + 1];
        double abz = w[pj + 2] - w[pi + 2];
        double acx = w[pk] - w[pi];
        double acy = w[pk + 1] - w[pi + 1];
        double acz = w[pk + 2] - w[pi + 2];
        double nx = aby * acz - abz * acy;
        double ny = abz * acx - abx * acz;
        double nz = abx * acy - aby * acx;
        double nn = nx * nx + ny * ny + nz * nz;
        if (nn > 0) {
            // Barycentric coordinates of the origin's projection onto the triangle's plane.
            double la = (nx * (w[pj + 1] * w[pk + 2] - w[pj + 2] * w[pk + 1])
                + ny * (w[pj + 2] * w[pk] - w[pj] * w[pk + 2]) + nz * (w[pj] * w[pk + 1] - w[pj + 1] * w[pk])) / nn;
            double lb = (nx * (w[pk + 1] * w[pi + 2] - w[pk + 2] * w[pi + 1])
                + ny * (w[pk + 2] * w[pi] - w[pk] * w[pi + 2]) + nz * (w[pk] * w[pi + 1] - w[pk + 1] * w[pi])) / nn;
            double lc = 1 - la - lb;
            if (la >= 0 && lb >= 0 && lc >= 0) {
                offer(i, la, j, lb, k, lc);
                return;
            }
        }
        segment(i, j);
        segment(j, k);
        segment(k, i);
    }

    private void tetrahedron() {
        double volume = det(1, 2, 3, 0);
        // Signed volumes with each vertex replaced by the origin, relative to the whole.
        double l0 = det0(1, 2, 3) / volume;
        double l1 = -det0(0, 2, 3) / volume;
        double l2 = det0(0, 1, 3) / volume;
        double l3 = 1 - l0 - l1 - l2;
        // A flat tetrahedron (volume 0) gives infinite or NaN weights, which never all pass.
        if (Math.min(Math.min(l0, l1), Math.min(l2, l3)) >= 0) {
            bestMask = 15;
            bestLambda[0] = l0;
            bestLambda[1] = l1;
            bestLambda[2] = l2;
            bestLambda[3] = l3;
            return;
        }
        triangle(0, 1, 2);
        triangle(0, 1, 3);
        triangle(0, 2, 3);
        triangle(1, 2, 3);
    }

    /** det(w_i - w_o, w_j - w_o, w_k - w_o). */
    private double det(int i, int j, int k, int o) {
        double ax = w[3 * i] - w[3 * o];
        double ay = w[3 * i + 1] - w[3 * o + 1];
        double az = w[3 * i + 2] - w[3 * o + 2];
        double bx = w[3 * j] - w[3 * o];
        double by = w[3 * j + 1] - w[3 * o + 1];
        double bz = w[3 * j + 2] - w[3 * o + 2];
        double cx = w[3 * k] - w[3 * o];
        double cy = w[3 * k + 1] - w[3 * o + 1];
        double cz = w[3 * k + 2] - w[3 * o + 2];
        return ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
    }

    /** det(w_i, w_j, w_k): the signed volume with the origin as fourth vertex. */
    private double det0(int i, int j, int k) {
        double ax = w[3 * i];
        double ay = w[3 * i + 1];
        double az = w[3 * i + 2];
        double bx = w[3 * j];
        double by = w[3 * j + 1];
        double bz = w[3 * j + 2];
        double cx = w[3 * k];
        double cy = w[3 * k + 1];
        double cz = w[3 * k + 2];
        return ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
    }

    /** Offers a candidate closest point given by up to three weighted vertices (index -1 for unused). */
    private void offer(int i, double li, int j, double lj, int k, double lk) {
        double px = li * w[3 * i] + lj * w[3 * j];
        double py = li * w[3 * i + 1] + lj * w[3 * j + 1];
        double pz = li * w[3 * i + 2] + lj * w[3 * j + 2];
        int mask = (li > 0 ? 1 << i : 0) | (lj > 0 ? 1 << j : 0);
        if (k >= 0) {
            px += lk * w[3 * k];
            py += lk * w[3 * k + 1];
            pz += lk * w[3 * k + 2];
            mask |= 1 << k; // triangle interior: all weights are non-negative
        }
        double distance = px * px + py * py + pz * pz;
        if (distance < bestDistance) {
            bestDistance = distance;
            bestMask = mask;
            bestLambda[i] = li;
            bestLambda[j] = lj;
            if (k >= 0) {
                bestLambda[k] = lk;
            }
        }
    }

    // ---------------------------------------------------------------- EPA

    /** Penetration of overlapping cores (EPA), plus the margins. */
    private double penetration(Contact contact) {
        closestPoints();
        for (int i = 0; i < 3 * count; i++) {
            ew[i] = w[i];
            ea[i] = a[i];
            eb[i] = b[i];
        }
        vertexCount = count;
        for (int d = 0; d < DIRECTIONS.length && vertexCount < 4; d += 3) {
            addIfIndependent(DIRECTIONS[d], DIRECTIONS[d + 1], DIRECTIONS[d + 2]);
        }
        if (vertexCount < 4) {
            return flatPenetration(contact); // the cores' difference has no volume
        }
        initialFaces();
        int best = -1;
        for (int iteration = 0; iteration < maxIterations; iteration++) {
            best = closestFace();
            int n = 3 * best;
            double nx = faceNormals[n];
            double ny = faceNormals[n + 1];
            double nz = faceNormals[n + 2];
            if (vertexCount == maxVertices) {
                break;
            }
            int v = addVertex(nx, ny, nz);
            int p = 3 * v;
            double gap = nx * ew[p] + ny * ew[p + 1] + nz * ew[p + 2] - faceDistances[best];
            if (gap <= EPA_TOLERANCE || !expand(v)) {
                break;
            }
        }
        return finish(best, contact);
    }

    private void addIfIndependent(double dx, double dy, double dz) {
        int v = vertexCount;
        int p = 3 * v;
        poseA.coreSupport(dx, dy, dz, ea, p);
        poseB.coreSupport(-dx, -dy, -dz, eb, p);
        ew[p] = ea[p] - eb[p];
        ew[p + 1] = ea[p + 1] - eb[p + 1];
        ew[p + 2] = ea[p + 2] - eb[p + 2];
        double ux = ew[p] - ew[0];
        double uy = ew[p + 1] - ew[1];
        double uz = ew[p + 2] - ew[2];
        double independence;
        if (v == 1) {
            independence = ux * ux + uy * uy + uz * uz;
        } else {
            double ex = ew[3] - ew[0];
            double ey = ew[4] - ew[1];
            double ez = ew[5] - ew[2];
            double cx = ey * uz - ez * uy;
            double cy = ez * ux - ex * uz;
            double cz = ex * uy - ey * ux;
            if (v == 2) {
                independence = cx * cx + cy * cy + cz * cz; // distance from the line, squared, times |e|^2
            } else {
                double fx = ew[6] - ew[0];
                double fy = ew[7] - ew[1];
                double fz = ew[8] - ew[2];
                double nx = ey * fz - ez * fy;
                double ny = ez * fx - ex * fz;
                double nz = ex * fy - ey * fx;
                double s = nx * ux + ny * uy + nz * uz;
                independence = s * s; // distance from the plane, squared, times |n|^2
            }
        }
        if (independence > INDEPENDENCE_EPSILON) {
            vertexCount++;
        }
    }

    /**
     * Penetration when the cores' Minkowski difference is a point, segment or flat polygon containing the origin:
     * the cores can be separated by a zero move perpendicular to it, so only the margins overlap.
     */
    private double flatPenetration(Contact contact) {
        double nx;
        double ny;
        double nz;
        if (vertexCount == 3) {
            nx = (ew[4] - ew[1]) * (ew[8] - ew[2]) - (ew[5] - ew[2]) * (ew[7] - ew[1]);
            ny = (ew[5] - ew[2]) * (ew[6] - ew[0]) - (ew[3] - ew[0]) * (ew[8] - ew[2]);
            nz = (ew[3] - ew[0]) * (ew[7] - ew[1]) - (ew[4] - ew[1]) * (ew[6] - ew[0]);
        } else {
            // Prefer the direction from A to B; for a segment, make it perpendicular.
            nx = poseB.centerX - poseA.centerX;
            ny = poseB.centerY - poseA.centerY;
            nz = poseB.centerZ - poseA.centerZ;
            if (vertexCount == 2) {
                double ex = ew[3] - ew[0];
                double ey = ew[4] - ew[1];
                double ez = ew[5] - ew[2];
                double f = (nx * ex + ny * ey + nz * ez) / (ex * ex + ey * ey + ez * ez);
                nx -= f * ex;
                ny -= f * ey;
                nz -= f * ez;
                if (nx * nx + ny * ny + nz * nz <= INDEPENDENCE_EPSILON) {
                    // Centers on the line: any perpendicular works.
                    double ax = Math.abs(ex) < Math.abs(ey) ? 1 : 0;
                    double ay = 1 - ax;
                    nx = -ez * ay;
                    ny = ez * ax;
                    nz = ex * ay - ey * ax;
                }
            } else if (nx * nx + ny * ny + nz * nz == 0) {
                ny = 1; // concentric: any direction works
            }
        }
        double length = Math.sqrt(nx * nx + ny * ny + nz * nz);
        write(contact, nx / length, ny / length, nz / length, 0);
        return contact.distance;
    }

    private int addVertex(double nx, double ny, double nz) {
        int v = vertexCount++;
        int p = 3 * v;
        poseA.coreSupport(nx, ny, nz, ea, p);
        poseB.coreSupport(-nx, -ny, -nz, eb, p);
        ew[p] = ea[p] - eb[p];
        ew[p + 1] = ea[p + 1] - eb[p + 1];
        ew[p + 2] = ea[p + 2] - eb[p + 2];
        return v;
    }

    private void initialFaces() {
        faceCount = 0;
        boolean flip = tetrahedronVolume() > 0; // vertex 3 is in front of face (0, 1, 2): wind the other way
        addFace(0, flip ? 2 : 1, flip ? 1 : 2);
        addFace(0, flip ? 1 : 3, flip ? 3 : 1);
        addFace(0, flip ? 3 : 2, flip ? 2 : 3);
        addFace(1, flip ? 2 : 3, flip ? 3 : 2);
    }

    /** Signed volume of the first four polytope vertices: positive when vertex 3 lies on the side face (0, 1, 2) faces. */
    private double tetrahedronVolume() {
        double ax = ew[3] - ew[0];
        double ay = ew[4] - ew[1];
        double az = ew[5] - ew[2];
        double bx = ew[6] - ew[0];
        double by = ew[7] - ew[1];
        double bz = ew[8] - ew[2];
        double cx = ew[9] - ew[0];
        double cy = ew[10] - ew[1];
        double cz = ew[11] - ew[2];
        return ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
    }

    /** Adds face (i, j, k) wound counter-clockwise seen from outside; returns false when out of room. */
    private boolean addFace(int i, int j, int k) {
        if (faceCount == maxFaces) {
            return false;
        }
        int f = faceCount++;
        faces[3 * f] = i;
        faces[3 * f + 1] = j;
        faces[3 * f + 2] = k;
        double abx = ew[3 * j] - ew[3 * i];
        double aby = ew[3 * j + 1] - ew[3 * i + 1];
        double abz = ew[3 * j + 2] - ew[3 * i + 2];
        double acx = ew[3 * k] - ew[3 * i];
        double acy = ew[3 * k + 1] - ew[3 * i + 1];
        double acz = ew[3 * k + 2] - ew[3 * i + 2];
        double nx = aby * acz - abz * acy;
        double ny = abz * acx - abx * acz;
        double nz = abx * acy - aby * acx;
        double length = Math.sqrt(nx * nx + ny * ny + nz * nz);
        // A degenerate face gets NaN normal and distance: never the closest face, never seen and removed.
        double inverse = 1 / length;
        faceNormals[3 * f] = nx * inverse;
        faceNormals[3 * f + 1] = ny * inverse;
        faceNormals[3 * f + 2] = nz * inverse;
        faceDistances[f] = (nx * ew[3 * i] + ny * ew[3 * i + 1] + nz * ew[3 * i + 2]) * inverse;
        faceAlive[f] = true;
        return true;
    }

    private int closestFace() {
        int best = 0;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int f = 0; f < faceCount; f++) {
            if (faceAlive[f] && faceDistances[f] < bestDistance) {
                bestDistance = faceDistances[f];
                best = f;
            }
        }
        return best;
    }

    /** Removes the faces vertex v can see and closes the hole with faces to v; false when out of room. */
    private boolean expand(int v) {
        int p = 3 * v;
        edgeCount = 0;
        for (int f = 0; f < faceCount; f++) {
            int n = 3 * f;
            int i = faces[n];
            double toVertex = faceNormals[n] * (ew[p] - ew[3 * i]) + faceNormals[n + 1] * (ew[p + 1] - ew[3 * i + 1])
                + faceNormals[n + 2] * (ew[p + 2] - ew[3 * i + 2]);
            if (faceAlive[f] && toVertex > 0) {
                faceAlive[f] = false;
                toggleEdge(faces[n], faces[n + 1]);
                toggleEdge(faces[n + 1], faces[n + 2]);
                toggleEdge(faces[n + 2], faces[n]);
            }
        }
        for (int e = 0; e < edgeCount; e++) {
            if (!addFace(edges[2 * e], edges[2 * e + 1], v)) {
                return false;
            }
        }
        return true;
    }

    /** Adds edge (i, j) to the horizon, or removes it if its reverse is there (shared by two removed faces). */
    private void toggleEdge(int i, int j) {
        for (int e = 0; e < edgeCount; e++) {
            if (edges[2 * e] == j && edges[2 * e + 1] == i) {
                edgeCount--;
                edges[2 * e] = edges[2 * edgeCount];
                edges[2 * e + 1] = edges[2 * edgeCount + 1];
                return;
            }
        }
        edges[2 * edgeCount] = i;
        edges[2 * edgeCount + 1] = j;
        edgeCount++;
    }

    /** Contact from the closest face: the origin's projection, mapped back to points on A and B. */
    private double finish(int face, Contact contact) {
        int n = 3 * face;
        double nx = faceNormals[n];
        double ny = faceNormals[n + 1];
        double nz = faceNormals[n + 2];
        double depth = faceDistances[face];
        int i = 3 * faces[n];
        int j = 3 * faces[n + 1];
        int k = 3 * faces[n + 2];
        // Barycentric coordinates of the projected origin (n * depth) in the face.
        double px = nx * depth - ew[i];
        double py = ny * depth - ew[i + 1];
        double pz = nz * depth - ew[i + 2];
        double e1x = ew[j] - ew[i];
        double e1y = ew[j + 1] - ew[i + 1];
        double e1z = ew[j + 2] - ew[i + 2];
        double e2x = ew[k] - ew[i];
        double e2y = ew[k + 1] - ew[i + 1];
        double e2z = ew[k + 2] - ew[i + 2];
        double d00 = e1x * e1x + e1y * e1y + e1z * e1z;
        double d01 = e1x * e2x + e1y * e2y + e1z * e2z;
        double d11 = e2x * e2x + e2y * e2y + e2z * e2z;
        double d20 = px * e1x + py * e1y + pz * e1z;
        double d21 = px * e2x + py * e2y + pz * e2z;
        double denominator = d00 * d11 - d01 * d01;
        double lj = (d11 * d20 - d01 * d21) / denominator;
        double lk = (d00 * d21 - d01 * d20) / denominator;
        double li = 1 - lj - lk;
        pax = li * ea[i] + lj * ea[j] + lk * ea[k];
        pay = li * ea[i + 1] + lj * ea[j + 1] + lk * ea[k + 1];
        paz = li * ea[i + 2] + lj * ea[j + 2] + lk * ea[k + 2];
        pbx = li * eb[i] + lj * eb[j] + lk * eb[k];
        pby = li * eb[i + 1] + lj * eb[j + 1] + lk * eb[k + 1];
        pbz = li * eb[i + 2] + lj * eb[j + 2] + lk * eb[k + 2];
        write(contact, nx, ny, nz, -depth);
        return contact.distance;
    }
}
