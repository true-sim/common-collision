package com.truesim.common.collision;

import com.truesim.common.math.geometry.Aabb;

/** A collision shape in its own local space; colliders place it in the world with a transform. */
public sealed interface Shape permits ConvexShape, TriangleMeshShape {

    /** Bounds in local space. */
    Aabb localBounds();
}
