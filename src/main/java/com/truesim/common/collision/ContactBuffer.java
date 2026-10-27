package com.truesim.common.collision;

/**
 * A fixed number of preallocated {@link Contact}s, filled by queries that find several contacts at once, such as a
 * convex shape against a mesh. When the buffer is full, a deeper contact replaces the shallowest one, so the buffer
 * always holds the most important contacts. Reuse one per thread so queries allocate nothing.
 */
public final class ContactBuffer {

    private final Contact[] contacts;
    private int size;

    public ContactBuffer(int capacity) {
        if (capacity < 1) {
            throw new IllegalArgumentException("Capacity must be positive: " + capacity);
        }
        contacts = new Contact[capacity];
        for (int i = 0; i < capacity; i++) {
            contacts[i] = new Contact();
        }
    }

    public int capacity() {
        return contacts.length;
    }

    public int size() {
        return size;
    }

    public boolean isEmpty() {
        return size == 0;
    }

    /** Contact {@code index}, {@code 0 <= index < size()}; valid until the buffer is filled again. */
    public Contact get(int index) {
        if (index >= size) {
            throw new IndexOutOfBoundsException("Index " + index + " out of bounds for size " + size);
        }
        return contacts[index];
    }

    /** The deepest (most penetrating, or closest) contact, or {@code null} when empty. */
    public Contact deepest() {
        Contact deepest = null;
        for (int i = 0; i < size; i++) {
            if (deepest == null || contacts[i].distance < deepest.distance) {
                deepest = contacts[i];
            }
        }
        return deepest;
    }

    public void clear() {
        size = 0;
    }

    /** Adds a copy of {@code contact}, replacing the shallowest one if full and the new one is deeper. */
    void add(Contact contact) {
        if (size < contacts.length) {
            contacts[size++].copyFrom(contact);
            return;
        }
        int shallowest = 0;
        for (int i = 1; i < size; i++) {
            if (contacts[i].distance > contacts[shallowest].distance) {
                shallowest = i;
            }
        }
        if (contact.distance < contacts[shallowest].distance) {
            contacts[shallowest].copyFrom(contact);
        }
    }
}
