package org.firstinspires.ftc.teamcode.common

import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d
import com.pedropathing.geometry.Pose
import java.util.*
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.round
import kotlin.math.sin

fun normalizeRadians(radians: Double): Double {
    return atan2(sin(radians), cos(radians))
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

fun Vec2d.fieldMirror(): Vec2d {
    return Vec2d(144.0 - this.x, this.y)
}

fun Double.fieldMirror(): Double {
    return normalizeDegrees(180.0 - this)
}

fun Pose2d.remainingAngleTo(other: Vec2d): Double {
    val angleTo = angleTo(other)
    return normalizeRadians(angleTo - radians)
}