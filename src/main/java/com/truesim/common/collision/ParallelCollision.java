package com.truesim.common.collision;

import com.truesim.annotations.AllowAllocation;
import com.truesim.common.jobs.JobSystem;
import java.util.Arrays;

/**
 * The broad and narrow phase of a {@link CollisionWorld} on all threads of a {@link JobSystem}, with exactly the
 * results of the single-threaded {@link CollisionWorld#findPairs} and {@link CollisionQuery#collide}: the same pairs in
 * the same order and the same contacts, bit for bit, on 1 or 100 threads.
 *
 * <ul>
 *   <li>Broad phase: the dynamic colliders are split into chunks; each chunk collects its pairs in its own buffer,
 *       and the buffers are joined in chunk order, which is the single-threaded order.
 *   <li>Narrow phase: the pairs are split into chunks; each pair writes only its own slots.
 * </ul>
 *
 * <pre>{@code
 * ParallelCollision parallel = new ParallelCollision(world, jobs);
 * world.update();
 * parallel.findPairs(pairs);
 * parallel.collide(pairs, contactDistance, manifolds);
 * }</pre>
 *
 * <p>Holds one {@link CollisionQuery} per thread and the chunk buffers, so steps allocate nothing once the buffers
 * have grown to fit. Use from one thread at a time, like the job system.
 */
public final class ParallelCollision {

    /** Dynamic colliders per broad-phase chunk. */
    private static final int PAIR_GRAIN = 32;
    /** Pairs per narrow-phase chunk. */
    private static final int CONTACT_GRAIN = 16;

    private final CollisionWorld world;
    private final JobSystem jobs;
    private final CollisionQuery[] queries;
    private PairBuffer[] chunkPairs = new PairBuffer[0];
    private final JobSystem.ChunkTask findChunk;
    private final JobSystem.RangeTask collideRange;

    // Parameters of the running step, read by the tasks.
    private PairBuffer pairs;
    private double maxDistance;
    private ContactManifolds manifolds;

    @AllowAllocation("one query per thread, and the tasks")
    public ParallelCollision(CollisionWorld world, JobSystem jobs) {
        this.world = world;
        this.jobs = jobs;
        this.queries = new CollisionQuery[jobs.threads()];
        for (int i = 0; i < queries.length; i++) {
            queries[i] = world.newQuery();
        }
        this.findChunk = this::findChunk;
        this.collideRange = this::collideRange;
    }

    /** Like {@link CollisionWorld#findPairs}, on all threads: the same pairs in the same order. */
    public void findPairs(PairBuffer out) {
        int dynamics = world.dynamicCount();
        int chunks = JobSystem.chunkCount(dynamics, PAIR_GRAIN);
        if (chunkPairs.length < chunks) {
            growChunks(chunks);
        }
        jobs.parallelChunks(dynamics, PAIR_GRAIN, findChunk);
        out.clear();
        for (int chunk = 0; chunk < chunks; chunk++) {
            out.addAll(chunkPairs[chunk]);
        }
    }

    @AllowAllocation("more dynamic colliders than ever before")
    private void growChunks(int chunks) {
        int old = chunkPairs.length;
        chunkPairs = Arrays.copyOf(chunkPairs, chunks);
        for (int chunk = old; chunk < chunks; chunk++) {
            chunkPairs[chunk] = new PairBuffer();
        }
    }

    private void findChunk(int chunk, int from, int to, int worker) {
        PairBuffer target = chunkPairs[chunk];
        target.clear();
        queries[worker].findPairs(from, to, target);
    }

    /**
     * Prepares {@code out} for all pairs and runs the narrow phase on all threads, like {@link CollisionQuery#collide}
     * over every pair: the same contacts in every slot.
     */
    public void collide(PairBuffer pairs, double maxDistance, ContactManifolds out) {
        out.prepare(pairs.size());
        this.pairs = pairs;
        this.maxDistance = maxDistance;
        this.manifolds = out;
        try {
            jobs.parallelFor(pairs.size(), CONTACT_GRAIN, collideRange);
        } finally {
            this.pairs = null;
            this.manifolds = null;
        }
    }

    private void collideRange(int from, int to, int worker) {
        queries[worker].collide(pairs, from, to, maxDistance, manifolds);
    }
}
