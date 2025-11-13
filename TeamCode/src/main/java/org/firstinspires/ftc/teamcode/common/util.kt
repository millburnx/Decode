package org.firstinspires.ftc.teamcode.common

import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d
import com.pedropathing.geometry.Pose
import java.util.*
import kotlin.math.floor
import kotlin.math.pow
import kotlin.math.round

fun normalizeRadians(radians: Double): Double {
    val temp = (radians + Math.PI) / (2.0 * Math.PI)
    return (temp - floor(temp) - 0.5) * 2.0
}

fun normalizeDegrees(angle: Double): Double = Math.toDegrees(normalizeRadians(Math.toRadians(angle)))

fun Pose2d.simplifiedString(): String = String.format(Locale.US, "(%.2f, %.2f, %.0f)", x, y, heading)

fun Vec2d.simplifiedString(): String = String.format(Locale.US, "(%.2f, %.2f)", x, y)

fun Double.roundTo(n: Int): Double {
    val factor = 10.0.pow(n)
    return round(this * factor) / factor
}

fun Pose2d.toPedro(): Pose {
    return Pose(this.x, this.y, this.radians)
}

fun Pose2d.fieldMirror(): Pose2d {
    return Pose2d(144.0 - this.x, this.y, normalizeDegrees(180.0 - this.degrees))
}

fun Pose2d.remainingAngleTo(other: Vec2d): Double {
    val angleTo = angleTo(other)
    return normalizeRadians(angleTo - radians)
}