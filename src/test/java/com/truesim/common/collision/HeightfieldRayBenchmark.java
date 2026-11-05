package com.truesim.common.collision;

import com.truesim.common.math.Transform;
import com.truesim.common.math.random.Rng;
import java.util.Arrays;
import java.util.Locale;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;

/**
 * What a raycast against a heightfield costs: a streamed terrain tile (64 x 64 cells of 0.5 m, as scene's cells use)
 * and a whole map (1024 x 1024 cells of 1 m), with short rays down (wheels, feet), long rays across (cameras, aiming:
 * hitting a slope far away, or passing over everything), and grazing rays that follow the ground just above it.
 * Prints the median over measured rounds after warm-up rounds, in ns per ray. Run with
 * {@code TRUESIM_BENCHMARK=true ./gradlew test --tests '*HeightfieldRayBenchmark'}.
 */
@EnabledIfEnvironmentVariable(named = "TRUESIM_BENCHMARK", matches = "true")
class HeightfieldRayBenchmark {

    private static final int RAYS = 4096;
    private static final int WARMUP = 10;
    private static final int ROUNDS = 15;

    /** Hills, valleys and bumps: octaves of smooth value noise, about {@code relief} high in all. */
    static HeightfieldShape terrain(int cells, double cellSize, double relief, long seed) {
        int samples = cells + 1;
        double[] heights = new double[samples * samples];
        Rng rng = Rng.seeded(seed);
        double amplitude = relief / 2;
        for (int octave = 0, period = Math.max(4, cells / 4); period >= 2; octave++, period /= 2, amplitude /= 2) {
            int grid = cells / period + 2;
            double[] lattice = new double[grid * grid];
            for (int i = 0; i < lattice.length; i++) {
                lattice[i] = rng.nextDouble(-1, 1);
            }
            for (int z = 0; z < samples; z++) {
                for (int x = 0; x < samples; x++) {
                    double gx = (double) x / period;
                    double gz = (double) z / period;
                    int ix = (int) gx;
                    int iz = (int) gz;
                    double fx = smooth(gx - ix);
                    double fz = smooth(gz - iz);
                    double a = lattice[iz * grid + ix] + fx * (lattice[iz * grid + ix + 1] - lattice[iz * grid + ix]);
                    double b = lattice[(iz + 1) * grid + ix]
                        + fx * (lattice[(iz + 1) * grid + ix + 1] - lattice[(iz + 1) * grid + ix]);
                    heights[z * samples + x] += amplitude * (a + fz * (b - a));
                }
            }
        }
        return HeightfieldShape.of(samples, samples, cellSize, cellSize, heights);
    }

    private static double smooth(double t) {
        return t * t * (3 - 2 * t);
    }

    /** Ground height under (x, z), from a straight-down raycast. */
    private static double ground(RayCaster caster, HeightfieldShape field, double x, double z, RaycastHit hit) {
        caster.cast(field, Transform.IDENTITY, x, 1e4, z, 0, -1, 0, 2e4, hit);
        return 1e4 - hit.distance();
    }

    /** Rays as origin, unit direction and length: 7 doubles each. */
    private static double[] rays(String kind, HeightfieldShape field, long seed) {
        RayCaster caster = new RayCaster();
        RaycastHit hit = new RaycastHit();
        Rng rng = Rng.seeded(seed);
        double size = field.cellsX() * field.cellSizeX();
        double top = field.localBounds().max().y();
        double[] rays = new double[7 * RAYS];
        for (int i = 0; i < RAYS; i++) {
            double x = rng.nextDouble(0.05, 0.95) * size;
            double z = rng.nextDouble(0.05, 0.95) * size;
            double angle = rng.nextDouble(0, 2 * Math.PI);
            double dx = Math.cos(angle);
            double dz = Math.sin(angle);
            double ox;
            double oy;
            double oz;
            double dy;
            double length;
            switch (kind) {
                case "short down" -> {
                    ox = x;
                    oz = z;
                    oy = ground(caster, field, x, z, hit) + 1;
                    dx = 0;
                    dz = 0;
                    dy = -1;
                    length = 2;
                }
                case "long, hits" -> {
                    // From above the terrain at one side, down at a shallow angle onto a point beyond the middle.
                    double tx = rng.nextDouble(0.4, 0.95) * size;
                    double tz = rng.nextDouble(0.4, 0.95) * size;
                    ox = tx - 0.9 * tx;
                    oz = tz - 0.9 * tz;
                    oy = top + 2;
                    double ty = ground(caster, field, tx, tz, hit);
                    double ex = tx - ox;
                    double ey = ty - oy;
                    double ez = tz - oz;
                    double l = Math.sqrt(ex * ex + ey * ey + ez * ez);
                    dx = ex / l;
                    dy = ey / l;
                    dz = ez / l;
                    length = 2 * l;
                }
                case "long, passes over" -> {
                    // From eye height above the ground, rising slowly: over valleys and hills, out of the field.
                    ox = x;
                    oz = z;
                    oy = ground(caster, field, x, z, hit) + 2;
                    dy = 0.03;
                    length = 2 * size;
                }
                default -> { // grazing: just above the ground, nearly level, across the field
                    ox = x;
                    oz = z;
                    oy = ground(caster, field, x, z, hit) + 0.05 * field.cellSizeX();
                    dy = 0.002;
                    length = 2 * size;
                }
            }
            double l = Math.sqrt(dx * dx + dy * dy + dz * dz);
            rays[7 * i] = ox;
            rays[7 * i + 1] = oy;
            rays[7 * i + 2] = oz;
            rays[7 * i + 3] = dx / l;
            rays[7 * i + 4] = dy / l;
            rays[7 * i + 5] = dz / l;
            rays[7 * i + 6] = length;
        }
        return rays;
    }

    @Test
    void raycasts() {
        String[] kinds = {"short down", "long, hits", "long, passes over", "grazing"};
        HeightfieldShape tile = terrain(64, 0.5, 4, 1);
        HeightfieldShape map = terrain(1024, 1, 60, 2);
        for (HeightfieldShape field : new HeightfieldShape[] {tile, map}) {
            for (String kind : kinds) {
                double[] rays = rays(kind, field, kind.hashCode());
                RayCaster caster = new RayCaster();
                RaycastHit hit = new RaycastHit();
                double[] samples = new double[ROUNDS];
                int hits = 0;
                for (int round = -WARMUP; round < ROUNDS; round++) {
                    hits = 0;
                    long t0 = System.nanoTime();
                    for (int i = 0; i < RAYS; i++) {
                        int r = 7 * i;
                        if (caster.cast(field, Transform.IDENTITY, rays[r], rays[r + 1], rays[r + 2], rays[r + 3],
                            rays[r + 4], rays[r + 5], rays[r + 6], hit)) {
                            hits++;
                        }
                    }
                    long t1 = System.nanoTime();
                    if (round >= 0) {
                        samples[round] = (double) (t1 - t0) / RAYS;
                    }
                }
                Arrays.sort(samples);
                System.out.printf(Locale.ROOT, "%4d x %-4d cells, %-17s: %8.0f ns per ray (best %.0f), %4.0f%% hit%n",
                    field.cellsX(), field.cellsZ(), kind, samples[ROUNDS / 2], samples[0], 100.0 * hits / RAYS);
            }
        }
    }
}
