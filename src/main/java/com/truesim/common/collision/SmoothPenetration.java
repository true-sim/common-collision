package com.truesim.common.collision;

/**
 * Exact signed distance between cores when one or both are smooth: non-uniformly scaled spheres and capsules, whose
 * curved surfaces EPA can only approximate with flat faces, and on which GJK converges only linearly.
 *
 * <p>The signed distance is minus the least h(n) over unit directions n (the penetration depth when positive), where
 * h is the support function of the cores' difference D = A - B. D is a polytope P (the sharp parts: points, segment
 * ends, box and hull corners, of A minus those of B) plus the smooth part E (A's ellipsoid plus B's), so
 * h(n) = h_P(n) + g(n) with g(n) = sum of r |K^T n| over the smooth cores (K = rotation times scale). g is smooth with
 * Hessian (r / |m|) K (I - m m^T / |m|^2) K^T, m = K^T n; h_P is piecewise linear, with kinks where the vertex of P
 * furthest along n changes. At the optimum, the set of P's furthest vertices is a vertex, an edge or a face:
 *
 * <ul>
 *   <li>vertex v: h(n) = v.n + g(n) is smooth; Newton's method on the unit sphere
 *   <li>edge v0-v1: n is perpendicular to the edge, on a great circle; Newton's method along it, and the point on the
 *       edge must lie between its ends
 *   <li>face v0-v1-v2: n is the face normal, no iteration; the point on the face must lie inside it
 * </ul>
 *
 * <p>EPA's closest face (or GJK's simplex, when the cores are apart) says which vertices of P are active to start with
 * (from the directions its corners came from); the refinement then adds a vertex P reaches beyond the current
 * feature, or drops one the point on the feature lies outside of, until the optimum is certified: P reaches no
 * further than the feature along n, and the support point of D at n lies along n. Newton's method converges
 * quadratically from EPA's normal, to rounding; where h is not curved upwards (far from the optimum, or nearly flat)
 * a Levenberg-Marquardt shift and a line search keep it going downhill.
 * The touching points are the support points at the optimum. The result is the optimum nearest EPA's: when two
 * directions need almost the same depth, it is within EPA's tolerance of the least. If the refinement does not
 * converge, EPA's (or GJK's) answer stands.
 */
final class SmoothPenetration {

    /** A Newton step this small (in radians) has converged to rounding. */
    private static final double STEP_TOLERANCE = 1e-13;
    /** The largest step, in radians. */
    private static final double MAX_STEP = 0.5;
    /** Steps this small are taken even if rounding makes h look higher: Newton's method has converged to rounding. */
    private static final double QUADRATIC = 1e-7;
    /** Relative slack for comparing how far vertices reach, and for weights of points on features. */
    private static final double TOLERANCE = 1e-12;

    private final Pose poseA;
    private final Pose poseB;
    /** Newton steps and feature changes in all, before giving up. */
    private final int maxSteps;

    // Active vertices of P: their points on A's and B's sharp parts, and the difference.
    private final double[] featureA = new double[9];
    private final double[] featureB = new double[9];
    private final double[] feature = new double[9];
    private int features;
    // The last sample of P: its points on A and B, and the difference in {@link #point}.
    private final double[] sampleA = new double[3];
    private final double[] sampleB = new double[3];
    private int lastSteps;

    // The smooth part at the current direction: offsets of A's and B's support points, and g.
    private double offAx;
    private double offAy;
    private double offAz;
    private double offBx;
    private double offBy;
    private double offBz;
    private double g;
    private final double[] mA = new double[3];
    private final double[] mB = new double[3];
    private final double[] u = new double[3];
    private final double[] point = new double[3];
    private double weight0;
    private double weight1;
    private double weight2;
    private double lambda;

    private double nx;
    private double ny;
    private double nz;
    private double t1x;
    private double t1y;
    private double t1z;
    private double t2x;
    private double t2y;
    private double t2z;

    // The result: normal from A to B and the touching points.
    double normalX;
    double normalY;
    double normalZ;
    double ax;
    double ay;
    double az;
    double bx;
    double by;
    double bz;

    SmoothPenetration(Pose poseA, Pose poseB, int maxSteps) {
        this.poseA = poseA;
        this.poseB = poseB;
        this.maxSteps = maxSteps;
    }

    /**
     * Refines EPA's normal ({@code n}) given the directions its closest face's corners came from ({@code count} of
     * them, three doubles each). Returns whether it found the exact optimum; then the result fields hold it.
     */
    boolean refine(double startX, double startY, double startZ, double[] directions, int count) {
        nx = startX;
        ny = startY;
        nz = startZ;
        features = 0;
        for (int i = 0; i < count; i++) {
            sample(directions[3 * i], directions[3 * i + 1], directions[3 * i + 2]);
            addFeature();
        }
        int steps = 0;
        while (steps < maxSteps) {
            steps++;
            if (features == 3) {
                if (!face()) {
                    removeFeature(2); // three points on a line: the edge case finds which of them matter
                    continue;
                }
                if (Math.min(weight0, Math.min(weight1, weight2)) < -TOLERANCE) {
                    removeFeature(leastWeight()); // the point lies outside, beyond the opposite edge
                    continue;
                }
            } else {
                boolean converged = descend(maxSteps - steps, features == 2);
                steps += lastSteps;
                if (!converged) {
                    return false;
                }
                if (features == 2 && (lambda < -TOLERANCE || lambda > 1 + TOLERANCE)) {
                    removeFeature(lambda < 0 ? 1 : 0); // the point lies beyond an end: that end alone is the feature
                    continue;
                }
            }
            // Does P reach further along n than the feature? Then that vertex joins it.
            sample(nx, ny, nz);
            if (beyond()) {
                if (features == 3) {
                    removeFeature(leastWeight()); // not a face of P: the new vertex replaces the least used corner
                }
                addFeature();
                continue;
            }
            finish();
            return true;
        }
        return false;
    }

    /** Whether the last sample reaches further along n than the active feature. */
    private boolean beyond() {
        double reach = point[0] * nx + point[1] * ny + point[2] * nz;
        double current = feature[0] * nx + feature[1] * ny + feature[2] * nz;
        return reach > current + TOLERANCE * (Math.abs(current) + norm(point));
    }

    private int leastWeight() {
        return weight0 <= weight1 && weight0 <= weight2 ? 0 : weight1 <= weight2 ? 1 : 2;
    }

    /** Samples P's furthest vertex along d, with its points on A and B. */
    private void sample(double dx, double dy, double dz) {
        poseA.sharpSupport(dx, dy, dz, sampleA, 0);
        poseB.sharpSupport(-dx, -dy, -dz, sampleB, 0);
        point[0] = sampleA[0] - sampleB[0];
        point[1] = sampleA[1] - sampleB[1];
        point[2] = sampleA[2] - sampleB[2];
    }

    /** Keeps the last sample as an active vertex, unless it is one already. */
    private void addFeature() {
        for (int i = 0; i < features; i++) {
            double x = feature[3 * i] - point[0];
            double y = feature[3 * i + 1] - point[1];
            double z = feature[3 * i + 2] - point[2];
            if (x * x + y * y + z * z == 0) {
                return;
            }
        }
        int at = 3 * features;
        System.arraycopy(sampleA, 0, featureA, at, 3);
        System.arraycopy(sampleB, 0, featureB, at, 3);
        feature[at] = point[0];
        feature[at + 1] = point[1];
        feature[at + 2] = point[2];
        features++;
    }

    private void removeFeature(int index) {
        for (int i = index; i < features - 1; i++) {
            System.arraycopy(featureA, 3 * i + 3, featureA, 3 * i, 3);
            System.arraycopy(featureB, 3 * i + 3, featureB, 3 * i, 3);
            System.arraycopy(feature, 3 * i + 3, feature, 3 * i, 3);
        }
        features--;
    }

    private static double norm(double[] p) {
        return Math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    }

    // ---------------------------------------------------------------- the smooth part

    /** The smooth part at the current n: the support offsets of A and B, and g. */
    private void round() {
        offAx = 0;
        offAy = 0;
        offAz = 0;
        offBx = 0;
        offBy = 0;
        offBz = 0;
        g = 0;
        if (poseA.smooth()) {
            g += offset(poseA, mA);
            offAx = u[0];
            offAy = u[1];
            offAz = u[2];
        }
        if (poseB.smooth()) {
            g += offset(poseB, mB);
            offBx = u[0];
            offBy = u[1];
            offBz = u[2];
        }
    }

    /** One smooth core's offset r K m / |m| (into {@link #u}) for m = K^T n (into {@code m}); returns r |m|. */
    private double offset(Pose pose, double[] m) {
        pose.localDirection(nx, ny, nz, m, 0);
        double length = Math.sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
        double r = pose.roundRadius();
        pose.worldVector(m[0] * r / length, m[1] * r / length, m[2] * r / length, u, 0);
        return r * length;
    }

    /** The smooth part's Hessian between tangent directions s and t at the current n. */
    private double hessian(double sx, double sy, double sz, double tx, double ty, double tz) {
        double sum = 0;
        if (poseA.smooth()) {
            sum += hessian(poseA, mA, sx, sy, sz, tx, ty, tz);
        }
        if (poseB.smooth()) {
            sum += hessian(poseB, mB, sx, sy, sz, tx, ty, tz);
        }
        return sum;
    }

    private double hessian(Pose pose, double[] m, double sx, double sy, double sz, double tx, double ty, double tz) {
        pose.localDirection(sx, sy, sz, u, 0);
        double usx = u[0];
        double usy = u[1];
        double usz = u[2];
        pose.localDirection(tx, ty, tz, u, 0);
        double mm = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
        double sm = usx * m[0] + usy * m[1] + usz * m[2];
        double tm = u[0] * m[0] + u[1] * m[1] + u[2] * m[2];
        return pose.roundRadius() / Math.sqrt(mm) * (usx * u[0] + usy * u[1] + usz * u[2] - sm * tm / mm);
    }

    // ---------------------------------------------------------------- features

    /** Two unit tangents at n, perpendicular to each other. */
    private void tangents() {
        // Cross n with the axis it is least aligned with.
        double ex = Math.abs(nx) <= Math.abs(ny) && Math.abs(nx) <= Math.abs(nz) ? 1 : 0;
        double ey = ex == 0 && Math.abs(ny) <= Math.abs(nz) ? 1 : 0;
        double ez = 1 - ex - ey;
        double x = ny * ez - nz * ey;
        double y = nz * ex - nx * ez;
        double z = nx * ey - ny * ex;
        double inverse = 1 / Math.sqrt(x * x + y * y + z * z);
        t1x = x * inverse;
        t1y = y * inverse;
        t1z = z * inverse;
        t2x = ny * t1z - nz * t1y;
        t2y = nz * t1x - nx * t1z;
        t2z = nx * t1y - ny * t1x;
    }

    private void setNormal(double x, double y, double z) {
        double inverse = 1 / Math.sqrt(x * x + y * y + z * z);
        nx = x * inverse;
        ny = y * inverse;
        nz = z * inverse;
    }

    /**
     * Newton's method for h(n) = v0.n + g(n): on the sphere for a vertex, or along the great circle perpendicular to
     * edge v0-v1, where v0.n = v1.n; on convergence also sets lambda, the place on the edge of the point that touches.
     * Where the curvature is not positive (far from the optimum, or where h is nearly flat) it shifts the curvature up
     * (Levenberg-Marquardt), and steps are halved until h goes down. False if it does not converge within the budget.
     */
    private boolean descend(int budget, boolean onEdge) {
        double ex = 0;
        double ey = 0;
        double ez = 0;
        double length = 1;
        if (onEdge) {
            ex = feature[3] - feature[0];
            ey = feature[4] - feature[1];
            ez = feature[5] - feature[2];
            length = Math.sqrt(ex * ex + ey * ey + ez * ez);
            ex /= length;
            ey /= length;
            ez /= length;
            ontoCircle(ex, ey, ez);
        }
        round();
        double value = value();
        lastSteps = 0;
        while (lastSteps < budget) {
            lastSteps++;
            double sx = feature[0] + offAx + offBx;
            double sy = feature[1] + offAy + offBy;
            double sz = feature[2] + offAz + offBz;
            if (onEdge) {
                t1x = ey * nz - ez * ny;
                t1y = ez * nx - ex * nz;
                t1z = ex * ny - ey * nx;
                t2x = 0;
                t2y = 0;
                t2z = 0;
            } else {
                tangents();
            }
            double b1 = t1x * sx + t1y * sy + t1z * sz;
            double b2 = t2x * sx + t2y * sy + t2z * sz;
            double a11 = hessian(t1x, t1y, t1z, t1x, t1y, t1z) - value;
            double a12 = onEdge ? 0 : hessian(t1x, t1y, t1z, t2x, t2y, t2z);
            double a22 = onEdge ? 1 : hessian(t2x, t2y, t2z, t2x, t2y, t2z) - value;
            double det = a11 * a22 - a12 * a12;
            if (!(a11 > 0 && det > 0)) {
                // Not curved upwards here (far from the optimum, or on a nearly flat h): shift the curvature up to
                // its opposite (Levenberg-Marquardt), which follows a direction of negative curvature far, and goes
                // downhill along the others.
                double least = (a11 + a22) / 2 - Math.sqrt((a11 - a22) * (a11 - a22) / 4 + a12 * a12);
                double shift = 2 * Math.abs(least) + TOLERANCE * (Math.abs(value) + 1);
                a11 += shift;
                a22 += onEdge ? 0 : shift;
                det = a11 * a22 - a12 * a12;
            }
            double step1 = (-b1 * a22 + b2 * a12) / det;
            double step2 = (-b2 * a11 + b1 * a12) / det;
            double size = Math.sqrt(step1 * step1 + step2 * step2);
            if (size > MAX_STEP) {
                step1 *= MAX_STEP / size;
                step2 *= MAX_STEP / size;
                size = MAX_STEP;
            }
            double fromX = nx;
            double fromY = ny;
            double fromZ = nz;
            while (true) {
                setNormal(fromX + step1 * t1x + step2 * t2x, fromY + step1 * t1y + step2 * t2y,
                    fromZ + step1 * t1z + step2 * t2z);
                if (onEdge) {
                    ontoCircle(ex, ey, ez);
                }
                round();
                double next = value();
                if (next <= value || size <= QUADRATIC) {
                    value = next;
                    break;
                }
                step1 /= 2;
                step2 /= 2;
                size /= 2;
            }
            if (size <= STEP_TOLERANCE) {
                lambda = -((feature[0] + offAx + offBx) * ex + (feature[1] + offAy + offBy) * ey
                    + (feature[2] + offAz + offBz) * ez) / length;
                return true;
            }
        }
        return false;
    }

    /** h(n) for the first active vertex, after {@link #round()}. */
    private double value() {
        return feature[0] * nx + feature[1] * ny + feature[2] * nz + g;
    }

    /** Moves n onto the great circle perpendicular to unit vector e. */
    private void ontoCircle(double ex, double ey, double ez) {
        double along = nx * ex + ny * ey + nz * ez;
        setNormal(nx - along * ex, ny - along * ey, nz - along * ez);
    }

    /** The face normal of v0-v1-v2 turned like n, and the weights of the point on the face; false if it is flat. */
    private boolean face() {
        lastSteps = 0;
        double e1x = feature[3] - feature[0];
        double e1y = feature[4] - feature[1];
        double e1z = feature[5] - feature[2];
        double e2x = feature[6] - feature[0];
        double e2y = feature[7] - feature[1];
        double e2z = feature[8] - feature[2];
        double cx = e1y * e2z - e1z * e2y;
        double cy = e1z * e2x - e1x * e2z;
        double cz = e1x * e2y - e1y * e2x;
        double cc = cx * cx + cy * cy + cz * cz;
        double e11 = e1x * e1x + e1y * e1y + e1z * e1z;
        double e22 = e2x * e2x + e2y * e2y + e2z * e2z;
        if (!(cc > TOLERANCE * TOLERANCE * e11 * e22)) {
            return false;
        }
        double turn = cx * nx + cy * ny + cz * nz < 0 ? -1 : 1;
        setNormal(turn * cx, turn * cy, turn * cz);
        round();
        double h = feature[0] * nx + feature[1] * ny + feature[2] * nz + g;
        // The point of P that, with the smooth part's support point, is the origin's foot on D at depth h.
        double px = h * nx - offAx - offBx - feature[0];
        double py = h * ny - offAy - offBy - feature[1];
        double pz = h * nz - offAz - offBz - feature[2];
        double e12 = e1x * e2x + e1y * e2y + e1z * e2z;
        double p1 = px * e1x + py * e1y + pz * e1z;
        double p2 = px * e2x + py * e2y + pz * e2z;
        double det = e11 * e22 - e12 * e12;
        weight1 = (e22 * p1 - e12 * p2) / det;
        weight2 = (e11 * p2 - e12 * p1) / det;
        weight0 = 1 - weight1 - weight2;
        return true;
    }

    /** The touching points at the optimum: the feature's point on A and B plus the smooth parts. */
    private void finish() {
        double w0;
        double w1;
        double w2;
        if (features == 1) {
            w0 = 1;
            w1 = 0;
            w2 = 0;
        } else if (features == 2) {
            w0 = 1 - lambda;
            w1 = lambda;
            w2 = 0;
        } else {
            w0 = weight0;
            w1 = weight1;
            w2 = weight2;
        }
        ax = offAx;
        ay = offAy;
        az = offAz;
        bx = -offBx;
        by = -offBy;
        bz = -offBz;
        for (int i = 0; i < features; i++) {
            double w = i == 0 ? w0 : i == 1 ? w1 : w2;
            ax += w * featureA[3 * i];
            ay += w * featureA[3 * i + 1];
            az += w * featureA[3 * i + 2];
            bx += w * featureB[3 * i];
            by += w * featureB[3 * i + 1];
            bz += w * featureB[3 * i + 2];
        }
        normalX = nx;
        normalY = ny;
        normalZ = nz;
    }
}
