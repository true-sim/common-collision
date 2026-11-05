plugins {
    id("truesim.java-library-conventions")
    id("truesim.allocation-free")
}

description = "Collision detection for true-sim: shapes, GJK/EPA, raycasts, contacts and a broad-phase collision world."

dependencies {
    api("com.truesim.common:common-math:0.3.0")
    api("com.truesim.common:common-jobs:0.1.0")
}

coverage {
    minimum = 1.0
}

// Benchmarks (TRUESIM_BENCHMARK=true) measure the code, not its coverage probes.
tasks.test {
    if (providers.environmentVariable("TRUESIM_BENCHMARK").orNull == "true") {
        maxHeapSize = "2g"
        extensions.configure<JacocoTaskExtension> { isEnabled = false }
    }
}
