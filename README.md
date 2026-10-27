# common-collision

Deterministic, allocation-free collision detection for true-sim, built on
[common-math](https://github.com/true-sim/common-math). Package `com.truesim.common.collision`.

Status: narrow phase between convex shapes, triangle meshes and heightfields. Planned next:
raycasts, contacts with triangles (milestone 2), a collision world with broad phase (milestone 3),
then shape casts (milestone 4).

## Usage

```kotlin
dependencies {
    implementation("com.truesim.common:common-collision:0.1.0")
}
```

```java
NarrowPhase narrow = new NarrowPhase(); // one per thread
Contact contact = new Contact();        // reused, filled by queries

ConvexShape car = new BoxShape(Vec3.of(0.9, 0.7, 2.2));
ConvexShape post = new CapsuleShape(1.0, 0.15);

double distance = narrow.query(car, carPose, post, postPose, contact);
if (distance < 0) {
    // Penetrating: moving the post by contact.depth() along contact.normal() separates them.
    resolve(contact.normal(), contact.depth(), contact.pointA(), contact.pointB());
}
boolean touching = narrow.overlaps(car, carPose, post, postPose); // cheaper yes/no
```

## Shapes

| Shape | Description |
|---|---|
| `SphereShape(radius)` | Sphere around the local origin |
| `CapsuleShape(halfLength, radius)` | Capsule along local Y |
| `BoxShape(halfExtents)` | Box centered on the local origin |
| `ConvexHullShape.of(points)` | Convex hull of a point list (tens of points) |
| `TriangleMeshShape.of(vertices, indices, materials)` | Triangle mesh with a material id per triangle and its own BVH, shared by every collider that uses it |
| `HeightfieldShape.of(samplesX, samplesZ, cellSizeX, cellSizeZ, heights, materials)` | Terrain grid with a material id per cell; the grid itself is the spatial index |

Meshes and heightfields are `ConcaveShape`s: `queryAabb` finds the triangles near a local-space
region, and each triangle has an index and a material id.

Shapes are placed with a `Transform` (translation, rotation, scale). Non-uniform scale is
supported: a scaled sphere is an ellipsoid.

## How it works

- Every convex shape is a *core* (point, segment, box, hull) plus a *margin* (the radius of round
  shapes). GJK computes the distance between the cores and the margins are added back, which is
  exact for spheres and capsules and converges in a few iterations.
- When the cores overlap, EPA finds the penetration depth and normal.
- `Contact` reports the signed distance (negative when penetrating), the unit normal from A to B
  and a point on each surface.
- All working memory is preallocated in `NarrowPhase`; queries allocate nothing. The thin query
  methods are registered for forced inlining with common-math's `JitInlining`, so transforms
  created by callers are removed by escape analysis too.
- No `java.lang.Math` beyond exactly rounded operations: results are identical on every platform.

## Building

```
./gradlew build
```

To develop against local checkouts of build-logic and common-math:

```
./gradlew build -Ptruesim.build-logic=../build-logic -Ptruesim.common-math=../common-math
```

The build enforces platform independence, allocation-free code (checked at build time and by
`AllocationFreeTest`) and 100% line and branch coverage.
