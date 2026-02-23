package org.firstinspires.ftc.teamcode.common.util

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.util.Pose2d
import com.millburnx.util.vector.Vec2d
import org.firstinspires.ftc.teamcode.common.util.geometry.Polygon
import kotlin.math.abs

@Configurable
object ZoneAssist {
    @JvmField
    var wallClearance = 18.0 / 2.0 // how much to shrink the zones near walls to account for the size of the bot

    val closeZone
        get() = Polygon.autoConvex(
            Vec2d(14, 128) + Vec2d(wallClearance, -wallClearance),
            Vec2d(22, 144) + Vec2d(wallClearance, -wallClearance),
            Vec2d(120, 144) + Vec2d(-wallClearance, -wallClearance),
            Vec2d(130, 128) + Vec2d(-wallClearance, -wallClearance),
            Vec2d(72, 72),
        )

    val farZone
        get() = Polygon.autoConvex(
            Vec2d(48, 0) + Vec2d(wallClearance, wallClearance),
            Vec2d(96, 0) + Vec2d(-wallClearance, wallClearance),
            Vec2d(72, 24),
        )

    @JvmField
    var strength = 0.15

    @JvmField
    var curve = 4.0 / 5.0

    @JvmField
    var epilsonBase = 2.0

    @JvmField
    var epilsonGain = 1.0

    fun calculateAssist(pos: Vec2d): Vec2d {
        val closeZone = this.closeZone
        val farZone = this.farZone

        if (closeZone.contains(pos) || farZone.contains(pos)) {
            return Vec2d(0.0, 0.0)
        }

        val closeZonePoint = closeZone.closestPoint(pos)
        val farZonePoint = farZone.closestPoint(pos)

        val closeZoneDistance = closeZonePoint.distance(pos)
        val farZoneDistance = farZonePoint.distance(pos)

        val targetPoint = if (closeZoneDistance <= farZoneDistance) closeZonePoint else farZonePoint

        val distance = targetPoint.distance(pos)

        val epilson = epilsonBase + epilsonGain * distance

        val deadzoneScale = (abs(closeZoneDistance - farZoneDistance) / epilson).clamp(0.0, 1.0)

        val baseVector = targetPoint - pos
        val deadzonedVector = baseVector * deadzoneScale
        val scaledVec = deadzonedVector.abs().pow(curve) * deadzonedVector.sign() * strength
        return scaledVec // field centric
    }

    fun calculateRelativeAssist(pose: Pose2d): Vec2d {
        val assist = calculateAssist(pose.position)
        return assist.rotate(-pose.radians)
    }
}