package com.truesim.common.collision;

import com.truesim.annotations.AllowAllocation;
import java.util.Arrays;

/**
 * Narrow-phase results for the pairs of a {@link PairBuffer}: up to {@link #contactsPerPair()} contacts per pair, each
 * oriented from the pair's first collider (A) to its second (B).
 *
 * <p>Every pair has its own slots, so workers can fill disjoint ranges of pairs in parallel and the result does not
 * depend on how the work was split or scheduled:
 *
 * <pre>{@code
 * world.findPairs(pairs);
 * manifolds.prepare(pairs.size());                    // one thread
 * query.collide(pairs, from, to, 0.02, manifolds);    // each worker: its own query and range
 * }</pre>
 */
public final class ContactManifolds {

    private final int contactsPerPair;
    private Contact[] contacts = new Contact[0];
    private int[] counts = new int[0];
    private int size;

    /** With room for {@code contactsPerPair} contacts per pair (the deepest are kept), e.g. 4. */
    public ContactManifolds(int contactsPerPair) {
        if (contactsPerPair < 1) {
            throw new IllegalArgumentException("Need room for at least one contact per pair: " + contactsPerPair);
        }
        this.contactsPerPair = contactsPerPair;
    }

    public int contactsPerPair() {
        return contactsPerPair;
    }

    /**
     * Makes room for {@code pairs} pairs and clears their counts. Call from one thread before workers collide ranges;
     * allocates only when the number of pairs exceeds every earlier one.
     */
    public void prepare(int pairs) {
        if (pairs > counts.length) {
            grow(pairs);
        }
        Arrays.fill(counts, 0, pairs, 0);
        size = pairs;
    }

    @AllowAllocation("growing, rare")
    private void grow(int pairs) {
        int capacity = Math.max(pairs, 2 * counts.length);
        int old = contacts.length;
        counts = Arrays.copyOf(counts, capacity);
        contacts = Arrays.copyOf(contacts, capacity * contactsPerPair);
        for (int i = old; i < contacts.length; i++) {
            contacts[i] = new Contact();
        }
    }

    /** Number of pairs prepared. */
    public int size() {
        return size;
    }

    /** Number of contacts found for {@code pair}. */
    public int count(int pair) {
        return counts[check(pair)];
    }

    /** Contact {@code index} ({@code < count(pair)}) of {@code pair}. */
    public Contact get(int pair, int index) {
        if (index < 0 || index >= counts[check(pair)]) {
            throw new IndexOutOfBoundsException("Contact " + index + " out of bounds for " + counts[pair] + " contacts");
        }
        return contacts[pair * contactsPerPair + index];
    }

    /** Total number of contacts over all pairs. */
    public int totalContacts() {
        int total = 0;
        for (int pair = 0; pair < size; pair++) {
            total += counts[pair];
        }
        return total;
    }

    private int check(int pair) {
        if (pair < 0 || pair >= size) {
            throw new IndexOutOfBoundsException("Pair " + pair + " out of bounds for size " + size);
        }
        return pair;
    }

    /** Stores {@code contact} (flipped if {@code flip}) as contact {@code index} of {@code pair}. */
    void set(int pair, int index, Contact contact, boolean flip) {
        Contact target = contacts[pair * contactsPerPair + index];
        if (flip) {
            target.copyFlipped(contact);
        } else {
            target.copyFrom(contact);
        }
    }

    void setCount(int pair, int count) {
        counts[pair] = count;
    }
}
