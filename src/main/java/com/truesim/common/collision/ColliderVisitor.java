package com.truesim.common.collision;

/** Visits colliders found by a {@link CollisionQuery}. Return {@code false} to stop the query. */
@FunctionalInterface
public interface ColliderVisitor {

    boolean visit(int collider);
}
