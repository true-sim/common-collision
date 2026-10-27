package com.truesim.common.collision;

/**
 * Classifies the edges between neighboring triangles. An edge is <em>active</em> when it is a real crease of the
 * surface: convex and bent by more than a few degrees, or on the boundary. Contacts on inactive edges (flat or concave
 * seams inside a surface) get the triangle's face normal instead, so objects sliding over a mesh do not catch on the
 * seams between its triangles.
 */
final class Edges {

    /** Edges bent by less than this angle's cosine (5 degrees) count as smooth. */
    static final double SMOOTH_COS = 0.9961946980917455;

    private Edges() {
    }

    /**
     * Whether edge a-b of triangle (a, b, c) is active, given the neighbor (b, a, d) across it. Both triangles wind
     * the same way, so the neighbor runs the shared edge backwards.
     */
    static boolean active(double ax, double ay, double az, double bx, double by, double bz, double cx, double cy,
        double cz, double dx, double dy, double dz) {
        double ex = bx - ax;
        double ey = by - ay;
        double ez = bz - az;
        // Normal of (a, b, c) and of the neighbor (b, a, d).
        double n1x = ey * (cz - az) - ez * (cy - ay);
        double n1y = ez * (cx - ax) - ex * (cz - az);
        double n1z = ex * (cy - ay) - ey * (cx - ax);
        double n2x = -ey * (dz - bz) + ez * (dy - by);
        double n2y = -ez * (dx - bx) + ex * (dz - bz);
        double n2z = -ex * (dy - by) + ey * (dx - bx);
        double lengths = Math.sqrt((n1x * n1x + n1y * n1y + n1z * n1z) * (n2x * n2x + n2y * n2y + n2z * n2z));
        if (lengths == 0) {
            return true; // a degenerate triangle: keep its edges sharp
        }
        boolean convex = n1x * (dx - ax) + n1y * (dy - ay) + n1z * (dz - az) < 0;
        return convex && n1x * n2x + n1y * n2y + n1z * n2z < SMOOTH_COS * lengths;
    }
}
