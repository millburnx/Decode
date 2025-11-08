package org.firstinspires.ftc.teamcode.common

import kotlin.math.floor

fun normalizeRadians(radians: Double): Double {
    val temp = (radians + Math.PI) / (2.0 * Math.PI)
    return (temp - floor(temp) - 0.5) * 2.0
}

fun normalizeDegrees(angle: Double): Double = Math.toDegrees(normalizeRadians(Math.toRadians(angle)))