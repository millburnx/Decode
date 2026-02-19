package org.firstinspires.ftc.teamcode.common.subsystem

import java.util.*

typealias Distance = Double
typealias RPM = Double
typealias Angle = Double

fun <K, V> treeMapOf(vararg pairs: Pair<K, V>): TreeMap<K, V> {
    return TreeMap<K, V>(mapOf(*pairs))
}

val NO_RECOIL_DATA: TreeMap<Distance, Pair<RPM, Angle>> = treeMapOf(
    36.0 to (2800.0 to 0.0),
    40.0 to (2700.0 to 0.0),
    44.0 to (2700.0 to 0.0),
    48.0 to (2800.0 to 0.0),
    64.0 to (2900.0 to 0.0),
    68.0 to (2900.0 to 0.0),
    72.0 to (3000.0 to 0.0),
    76.0 to (3000.0 to 0.0),
    80.0 to (3100.0 to 0.1),
    84.0 to (3200.0 to 0.1),
)

val FAR_ZONE: Pair<RPM, Angle> = (4000.0 to .4)

val CLOSE_DATA: TreeMap<Distance, TreeMap<RPM, Angle>> = treeMapOf(
    43.0 to treeMapOf(
        4300.0 to 0.0
    ),

    52.0 to treeMapOf(
        2600.0 to 0.0,
        2800.0 to 0.1,
        2900.0 to 0.2,
        3000.0 to 0.3
    ),

    60.0 to treeMapOf(
        2700.0 to 0.1,
        2900.0 to 0.2,
        3000.0 to 0.3,
        3200.0 to 0.4,
        3400.0 to 0.5
    ),

    66.0 to treeMapOf(
        2800.0 to 0.0,
        2900.0 to 0.1,
        3000.0 to 0.2,
        3100.0 to 0.35,
        3200.0 to 0.4,
        3400.0 to 0.5,
        3700.0 to 0.6
    ),

    72.0 to treeMapOf(
        3000.0 to 0.0,
        3100.0 to 0.1,
        3200.0 to 0.3,
        3300.0 to 0.4,
        3400.0 to 0.5,
        3600.0 to 0.6,
        3800.0 to 0.7,
        4000.0 to 0.8
    )
)
val FAR_DATA: TreeMap<RPM, Angle> = treeMapOf(
    3800.0 to 0.1,
    3900.0 to 0.4,
    4000.0 to 0.5,
    4100.0 to 0.6,
    4300.0 to 0.7,
    4500.0 to 0.8,
    4600.0 to 0.9,
    5000.0 to 1.0,
)

val FAR_DISTANCE: Distance = 120.0