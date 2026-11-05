package com.truesim.common.collision;

import com.truesim.common.math.Vec3;
import com.truesim.common.math.geometry.Aabb;
import com.truesim.common.math.geometry.Triangle;
import com.truesim.common.math.spatial.Bvh;
import java.util.Arrays;

/**
 * Terrain as a regular grid of heights, with a material id per cell.
 *
 * <p>Sample {@code (x, z)} sits at local position {@code (x * cellSizeX, height(x, z), z * cellSizeZ)}: the grid
 * starts at the local origin and extends along +X and +Z, with heights along Y. Each cell is split into two triangles
 * along the same diagonal, so cell {@code (x, z)} holds triangles {@code 2 * cell} and {@code 2 * cell + 1}, where
 * {@code cell = z * cellsX() + x}:
 *
 * <ul>
 *   <li>triangle {@code 2 * cell}: samples {@code (x, z)}, {@code (x, z + 1)}, {@code (x + 1, z)}
 *   <li>triangle {@code 2 * cell + 1}: samples {@code (x + 1, z)}, {@code (x, z + 1)}, {@code (x + 1, z + 1)}
 * </ul>
 *
 * <p>The grid itself is the spatial index: queries look only at the cells under the query region. Raycasts also skip
 * whole blocks of cells they pass above or below, by the height bounds of square blocks of 2, 4, 8 ... cells built
 * with the heightfield ({@link #withBlockBounds(boolean)}). Immutable; queries are read-only and thread-safe.
 */
public final class HeightfieldShape implements ConcaveShape {

    private final int samplesX;
    private final int samplesZ;
    private final double cellSizeX;
    private final double cellSizeZ;
    private final double[] heights;
    private final int[] materials;
    private final Aabb bounds;
    /**
     * Height bounds of square blocks of cells, so raycasts can skip whole blocks they pass above or below: level
     * {@code l} (from 1) holds blocks of 2^l x 2^l cells, min and max interleaved, rounded outwards to floats. Level 0,
     * the cells themselves, is read from the heights.
     */
    private final float[][] blockBounds;
    private final boolean blocks;

    private HeightfieldShape(int samplesX, int samplesZ, double cellSizeX, double cellSizeZ, double[] heights,
        int[] materials, boolean blocks) {
        this.samplesX = samplesX;
        this.samplesZ = samplesZ;
        this.cellSizeX = cellSizeX;
        this.cellSizeZ = cellSizeZ;
        this.heights = heights;
        this.materials = materials;
        this.blocks = blocks;
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (double h : heights) {
            min = Math.min(min, h);
            max = Math.max(max, h);
        }
        this.bounds = new Aabb(Vec3.of(0, min, 0), Vec3.of((samplesX - 1) * cellSizeX, max, (samplesZ - 1) * cellSizeZ));
        int levels = 0;
        while (blocks && (blocksX(levels) > 1 || blocksZ(levels) > 1)) {
            levels++;
        }
        this.blockBounds = new float[levels + 1][];
        for (int level = 1; level <= levels; level++) {
            buildLevel(level);
        }
    }

    /** One level of block bounds, from the samples (level 1) or from the level below. */
    private void buildLevel(int level) {
        int blocksX = blocksX(level);
        int blocksZ = blocksZ(level);
        float[] bounds = new float[2 * blocksX * blocksZ];
        for (int bz = 0; bz < blocksZ; bz++) {
            for (int bx = 0; bx < blocksX; bx++) {
                double min = Double.POSITIVE_INFINITY;
                double max = Double.NEGATIVE_INFINITY;
                if (level == 1) {
                    // The samples at the corners of the block's (up to) 2 x 2 cells.
                    for (int z = 2 * bz; z <= Math.min(2 * bz + 2, samplesZ - 1); z++) {
                        for (int x = 2 * bx; x <= Math.min(2 * bx + 2, samplesX - 1); x++) {
                            min = Math.min(min, heights[z * samplesX + x]);
                            max = Math.max(max, heights[z * samplesX + x]);
                        }
                    }
                } else {
                    // The (up to) 2 x 2 blocks of the level below.
                    for (int z = 2 * bz; z <= Math.min(2 * bz + 1, blocksZ(level - 1) - 1); z++) {
                        for (int x = 2 * bx; x <= Math.min(2 * bx + 1, blocksX(level - 1) - 1); x++) {
                            min = Math.min(min, blockMin(level - 1, x, z));
                            max = Math.max(max, blockMax(level - 1, x, z));
                        }
                    }
                }
                bounds[2 * (bz * blocksX + bx)] = roundDown(min);
                bounds[2 * (bz * blocksX + bx) + 1] = roundUp(max);
            }
        }
        blockBounds[level] = bounds;
    }

    private static float roundDown(double value) {
        float rounded = (float) value;
        return rounded > value ? Math.nextDown(rounded) : rounded;
    }

    private static float roundUp(double value) {
        float rounded = (float) value;
        return rounded < value ? Math.nextUp(rounded) : rounded;
    }

    /**
     * This heightfield with or without block bounds, sharing its heights and materials. With them (the default),
     * raycasts skip whole blocks of cells they pass above or below: on a 1024 x 1024 map, long rays are 2.7 to 5.7
     * times faster and rays grazing the ground 2 times; on a tile of 64 x 64 cells, short and long rays are 1.1 to 1.4
     * times faster. Without them, raycasts walk every cell under the ray, which is about 10% faster for rays grazing a
     * small tile, and the heightfield needs a third less memory. See {@code HeightfieldRayBenchmark}.
     */
    public HeightfieldShape withBlockBounds(boolean blocks) {
        return blocks == hasBlockBounds() ? this
            : new HeightfieldShape(samplesX, samplesZ, cellSizeX, cellSizeZ, heights, materials, blocks);
    }

    /** Whether raycasts skip blocks of cells by their height bounds; see {@link #withBlockBounds(boolean)}. */
    public boolean hasBlockBounds() {
        return blocks;
    }

    /** Levels of block bounds above the cells; on the top level, one block covers the whole grid. */
    int blockLevels() {
        return blockBounds.length - 1;
    }

    /** Blocks along x on a level: 2^level cells each, the last one possibly fewer. Level 0 is the cells. */
    int blocksX(int level) {
        return ((samplesX - 2) >> level) + 1;
    }

    /** Blocks along z on a level: 2^level cells each, the last one possibly fewer. Level 0 is the cells. */
    int blocksZ(int level) {
        return ((samplesZ - 2) >> level) + 1;
    }

    /** A lower bound of the heights in block (bx, bz) of a level from 1 up. */
    double blockMin(int level, int bx, int bz) {
        return blockBounds[level][2 * (bz * blocksX(level) + bx)];
    }

    /** An upper bound of the heights in block (bx, bz) of a level from 1 up. */
    double blockMax(int level, int bx, int bz) {
        return blockBounds[level][2 * (bz * blocksX(level) + bx) + 1];
    }

    /** A heightfield with material id 0 in every cell. */
    public static HeightfieldShape of(int samplesX, int samplesZ, double cellSizeX, double cellSizeZ, double[] heights) {
        return of(samplesX, samplesZ, cellSizeX, cellSizeZ, heights, new int[Math.max(0, (samplesX - 1) * (samplesZ - 1))]);
    }

    /**
     * A heightfield of {@code samplesX} by {@code samplesZ} samples (at least 2 each), with {@code heights} in rows of
     * {@code samplesX} (index {@code z * samplesX + x}) and one material id per cell (index {@code z * cellsX + x}).
     * The arrays are copied.
     */
    public static HeightfieldShape of(
        int samplesX, int samplesZ, double cellSizeX, double cellSizeZ, double[] heights, int[] materials) {
        if (samplesX < 2 || samplesZ < 2 || !(cellSizeX > 0) || !(cellSizeZ > 0)) {
            throw new IllegalArgumentException("Need at least 2 x 2 samples and positive cell sizes: " + samplesX
                + " x " + samplesZ + ", " + cellSizeX + " x " + cellSizeZ);
        }
        if (heights.length != samplesX * samplesZ || materials.length != (samplesX - 1) * (samplesZ - 1)) {
            throw new IllegalArgumentException("Need " + samplesX * samplesZ + " heights and "
                + (samplesX - 1) * (samplesZ - 1) + " materials: " + heights.length + ", " + materials.length);
        }
        for (double h : heights) {
            if (!Double.isFinite(h)) {
                throw new IllegalArgumentException("Heights must be finite: " + h);
            }
        }
        return new HeightfieldShape(samplesX, samplesZ, cellSizeX, cellSizeZ,
            Arrays.copyOf(heights, heights.length), Arrays.copyOf(materials, materials.length), true);
    }

    public int samplesX() {
        return samplesX;
    }

    public int samplesZ() {
        return samplesZ;
    }

    public int cellsX() {
        return samplesX - 1;
    }

    public int cellsZ() {
        return samplesZ - 1;
    }

    public double cellSizeX() {
        return cellSizeX;
    }

    public double cellSizeZ() {
        return cellSizeZ;
    }

    public double height(int x, int z) {
        return heights[z * samplesX + x];
    }

    /** Material id of cell {@code (x, z)}. */
    public int cellMaterial(int x, int z) {
        return materials[z * (samplesX - 1) + x];
    }

    @Override
    public int triangleCount() {
        return 2 * (samplesX - 1) * (samplesZ - 1);
    }

    @Override
    public int material(int triangle) {
        return materials[triangle >> 1];
    }

    @Override
    public void triangle(int triangle, double[] out, int at) {
        int cell = triangle >> 1;
        int x = cell % (samplesX - 1);
        int z = cell / (samplesX - 1);
        if ((triangle & 1) == 0) {
            sample(x, z, out, at);
            sample(x, z + 1, out, at + 3);
            sample(x + 1, z, out, at + 6);
        } else {
            sample(x + 1, z, out, at);
            sample(x, z + 1, out, at + 3);
            sample(x + 1, z + 1, out, at + 6);
        }
    }

    public Triangle triangle(int triangle) {
        double[] p = new double[9];
        triangle(triangle, p, 0);
        return new Triangle(Vec3.of(p[0], p[1], p[2]), Vec3.of(p[3], p[4], p[5]), Vec3.of(p[6], p[7], p[8]));
    }

    @Override
    public int activeEdges(int triangle) {
        int cell = triangle >> 1;
        int x = cell % (samplesX - 1);
        int z = cell / (samplesX - 1);
        if ((triangle & 1) == 0) {
            // a = (x, z), b = (x, z + 1), c = (x + 1, z); neighbors across a-b, b-c (the diagonal) and c-a.
            return edge(x, z, x, z + 1, x + 1, z, x - 1, z + 1, 0)
                | edge(x, z + 1, x + 1, z, x, z, x + 1, z + 1, 1)
                | edge(x + 1, z, x, z, x, z + 1, x + 1, z - 1, 2);
        }
        // a = (x + 1, z), b = (x, z + 1), c = (x + 1, z + 1); neighbors across a-b (the diagonal), b-c and c-a.
        return edge(x + 1, z, x, z + 1, x + 1, z + 1, x, z, 0)
            | edge(x, z + 1, x + 1, z + 1, x + 1, z, x, z + 2, 1)
            | edge(x + 1, z + 1, x + 1, z, x, z + 1, x + 2, z, 2);
    }

    /** Bit {@code bit} if edge (a, b) of triangle (a, b, c) is active given the neighbor's far sample d, which may lie outside the grid. */
    private int edge(int ax, int az, int bx, int bz, int cx, int cz, int dx, int dz, int bit) {
        if (dx < 0 || dx >= samplesX || dz < 0 || dz >= samplesZ) {
            return 1 << bit; // boundary
        }
        boolean active = Edges.active(ax * cellSizeX, height(ax, az), az * cellSizeZ, bx * cellSizeX, height(bx, bz),
            bz * cellSizeZ, cx * cellSizeX, height(cx, cz), cz * cellSizeZ, dx * cellSizeX, height(dx, dz),
            dz * cellSizeZ);
        return active ? 1 << bit : 0;
    }

    private void sample(int x, int z, double[] out, int at) {
        out[at] = x * cellSizeX;
        out[at + 1] = heights[z * samplesX + x];
        out[at + 2] = z * cellSizeZ;
    }

    @Override
    public Aabb localBounds() {
        return bounds;
    }

    @Override
    public int queryAabb(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, Bvh.Visitor visitor) {
        // Cells touching [min, max]; a box exactly on a cell edge touches the cells on both sides.
        int x0 = Math.max(0, (int) Math.ceil(minX / cellSizeX) - 1);
        int z0 = Math.max(0, (int) Math.ceil(minZ / cellSizeZ) - 1);
        int x1 = Math.min(samplesX - 2, (int) Math.floor(maxX / cellSizeX));
        int z1 = Math.min(samplesZ - 2, (int) Math.floor(maxZ / cellSizeZ));
        int visited = 0;
        for (int z = z0; z <= z1; z++) {
            for (int x = x0; x <= x1; x++) {
                double h00 = heights[z * samplesX + x];
                double h10 = heights[z * samplesX + x + 1];
                double h01 = heights[(z + 1) * samplesX + x];
                double h11 = heights[(z + 1) * samplesX + x + 1];
                int cell = z * (samplesX - 1) + x;
                // Both triangles span the whole cell in x and z, so only their heights decide.
                if (Math.max(h00, Math.max(h01, h10)) >= minY && Math.min(h00, Math.min(h01, h10)) <= maxY) {
                    visited++;
                    if (!visitor.visit(2 * cell)) {
                        return visited;
                    }
                }
                if (Math.max(h10, Math.max(h01, h11)) >= minY && Math.min(h10, Math.min(h01, h11)) <= maxY) {
                    visited++;
                    if (!visitor.visit(2 * cell + 1)) {
                        return visited;
                    }
                }
            }
        }
        return visited;
    }
}
