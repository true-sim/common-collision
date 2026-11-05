package com.truesim.common.collision;

import com.truesim.common.math.Transform;
import com.truesim.common.math.Vec3;
import com.truesim.common.math.random.Rng;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;

/**
 * What a narrow-phase query costs, and how exact its penetration is, for pairs of shapes with and without smooth cores
 * (non-uniformly scaled spheres and capsules), overlapping or apart. Exactness: A moved out by the depth along the
 * normal should just touch B; the error is the distance left. Prints the median over measured rounds after warm-up
 * rounds. Run with {@code TRUESIM_BENCHMARK=true ./gradlew test --tests '*NarrowPhaseBenchmark'}.
 */
@EnabledIfEnvironmentVariable(named = "TRUESIM_BENCHMARK", matches = "true")
class NarrowPhaseBenchmark {

    private static final int PAIRS = 2048;
    private static final int WARMUP = 10;
    private static final int ROUNDS = 15;

    private record Pair(ConvexShape a, Transform ta, ConvexShape b, Transform tb) {
    }

    private static Vec3 stretch(Rng rng) {
        return Vec3.of(rng.nextDouble(0.5, 2), rng.nextDouble(0.5, 2), rng.nextDouble(0.5, 2));
    }

    /** Random pairs of the kind, overlapping or apart; the shapes are about a unit in size. */
    private static List<Pair> pairs(String kind, boolean overlapping, long seed) {
        Rng rng = Rng.seeded(seed);
        NarrowPhase narrow = new NarrowPhase();
        Contact contact = new Contact();
        List<Pair> pairs = new ArrayList<>();
        while (pairs.size() < PAIRS) {
            ConvexShape a;
            ConvexShape b;
            Vec3 sa = Vec3.ONE;
            Vec3 sb = Vec3.ONE;
            switch (kind) {
                case "ellipsoid / ellipsoid" -> {
                    a = new SphereShape(0.6);
                    b = new SphereShape(0.5);
                    sa = stretch(rng);
                    sb = stretch(rng);
                }
                case "ellipsoid / box" -> {
                    a = new SphereShape(0.6);
                    b = new BoxShape(Vec3.of(0.8, 0.5, 0.6));
                    sa = stretch(rng);
                }
                case "scaled capsule / box" -> {
                    a = new CapsuleShape(0.6, 0.3);
                    b = new BoxShape(Vec3.of(0.8, 0.5, 0.6));
                    sa = stretch(rng);
                }
                case "scaled capsules" -> {
                    a = new CapsuleShape(0.6, 0.3);
                    b = new CapsuleShape(0.4, 0.4);
                    sa = stretch(rng);
                    sb = stretch(rng);
                }
                case "box / box" -> {
                    a = new BoxShape(Vec3.of(0.5, 0.7, 0.4));
                    b = new BoxShape(Vec3.of(0.8, 0.5, 0.6));
                }
                default -> { // sphere / box, uniform: a point core with a margin
                    a = new SphereShape(0.6);
                    b = new BoxShape(Vec3.of(0.8, 0.5, 0.6));
                }
            }
            Transform ta = Transform.of(rng.nextVec3InSphere().scale(1.5), rng.nextRotation(), sa);
            Transform tb = Transform.of(rng.nextVec3InSphere().scale(1.5), rng.nextRotation(), sb);
            if (narrow.query(a, ta, b, tb, contact) < 0 == overlapping) {
                pairs.add(new Pair(a, ta, b, tb));
            }
        }
        return pairs;
    }

    @Test
    void queries() {
        String[] kinds = {"ellipsoid / ellipsoid", "ellipsoid / box", "scaled capsule / box", "scaled capsules",
            "box / box", "sphere / box"};
        NarrowPhase narrow = new NarrowPhase();
        NarrowPhase check = new NarrowPhase();
        Contact contact = new Contact();
        Contact touch = new Contact();
        for (boolean overlapping : new boolean[] {true, false}) {
            for (String kind : kinds) {
                List<Pair> pairs = pairs(kind, overlapping, kind.hashCode());
                double[] samples = new double[ROUNDS];
                double sum = 0;
                for (int round = -WARMUP; round < ROUNDS; round++) {
                    sum = 0;
                    long t0 = System.nanoTime();
                    for (Pair pair : pairs) {
                        sum += narrow.query(pair.a(), pair.ta(), pair.b(), pair.tb(), contact);
                    }
                    long t1 = System.nanoTime();
                    if (round >= 0) {
                        samples[round] = (double) (t1 - t0) / PAIRS;
                    }
                }
                Arrays.sort(samples);
                double worst = 0;
                if (overlapping) {
                    for (Pair pair : pairs) {
                        double d = narrow.query(pair.a(), pair.ta(), pair.b(), pair.tb(), contact);
                        Vec3 moved = pair.ta().translation().add(contact.normal().scale(d));
                        Transform out = pair.ta().withTranslation(moved);
                        worst = Math.max(worst, Math.abs(check.query(pair.a(), out, pair.b(), pair.tb(), touch)));
                    }
                }
                System.out.printf(Locale.ROOT, "%-11s %-21s: %6.0f ns per query (best %.0f), moved out by the depth: "
                    + "%.1e apart at most, sum %.12e%n", overlapping ? "overlapping" : "apart", kind,
                    samples[ROUNDS / 2], samples[0], worst, sum);
            }
        }
    }
}
