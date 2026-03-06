package org.firstinspires.ftc.teamcode.common.subsystem

import java.util.*

typealias Distance = Double
typealias RPM = Double
typealias Angle = Double

fun <K, V> treeMapOf(vararg pairs: Pair<K, V>): TreeMap<K, V> {
    return TreeMap<K, V>(mapOf(*pairs))
}

val DATA: TreeMap<Distance, Pair<RPM, Angle>> = treeMapOf(
    40.0 to (3000.0 to 0.0),
    44.0 to (3000.0 to 0.0),
    48.0 to (2900.0 to 0.0),
    52.0 to (2900.0 to 0.0),
    56.0 to (3000.0 to 0.0),
    60.0 to (3000.0 to 0.0),
    64.0 to (3000.0 to 0.0),
    68.0 to (3100.0 to 0.0),
    72.0 to (3100.0 to 0.0),
    76.0 to (3100.0 to 0.0),
    80.0 to (3200.0 to 0.0),
    84.0 to (3200.0 to 0.0),
    88.0 to (3300.0 to 0.0),
    92.0 to (3300.0 to 0.0),
    96.0 to (3400.0 to 0.0),
    100.0 to (3400.0 to 0.0),
    104.0 to (3500.0 to 0.0),
    108.0 to (3500.0 to 0.0),
    112.0 to (3600.0 to 0.0),
    116.0 to (3600.0 to 0.0),
    120.0 to (3700.0 to 0.0),
    124.0 to (3800.0 to 0.0),
    128.0 to (3800.0 to 0.0),
    132.0 to (3900.0 to 0.0),
    136.0 to (4000.0 to 0.0),
    140.0 to (4000.0 to 0.0),
    144.0 to (4100.0 to 0.0),
    148.0 to (4200.0 to 0.0),
)