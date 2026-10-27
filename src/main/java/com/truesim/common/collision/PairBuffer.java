package com.truesim.common.collision;

import com.truesim.annotations.AllowAllocation;
import java.util.Arrays;

/**
 * Pairs of colliders found by {@link CollisionWorld#findPairs}. Grows as needed, so after the first few steps finding
 * pairs allocates nothing. Pair {@code i} is ({@link #first}, {@link #second}), in a deterministic order.
 */
public final class PairBuffer {

    private int[] pairs;
    private int size;

    public PairBuffer() {
        this(64);
    }

    public PairBuffer(int initialCapacity) {
        pairs = new int[2 * Math.max(1, initialCapacity)];
    }

    public int size() {
        return size;
    }

    public int first(int pair) {
        return pairs[2 * check(pair)];
    }

    public int second(int pair) {
        return pairs[2 * check(pair) + 1];
    }

    private int check(int pair) {
        if (pair < 0 || pair >= size) {
            throw new IndexOutOfBoundsException("Pair " + pair + " out of bounds for size " + size);
        }
        return pair;
    }

    public void clear() {
        size = 0;
    }

    void add(int first, int second) {
        if (2 * size == pairs.length) {
            grow();
        }
        pairs[2 * size] = first;
        pairs[2 * size + 1] = second;
        size++;
    }

    @AllowAllocation("growing, rare")
    private void grow() {
        pairs = Arrays.copyOf(pairs, 2 * pairs.length);
    }
}
