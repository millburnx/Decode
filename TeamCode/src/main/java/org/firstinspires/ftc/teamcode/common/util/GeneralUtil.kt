package org.firstinspires.ftc.teamcode.common.util

import kotlin.math.abs

fun Double.deadzone(min: Double) = if (abs(this) < min) 0.0 else this