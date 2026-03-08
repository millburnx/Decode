package org.firstinspires.ftc.teamcode.common.util

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import org.firstinspires.ftc.teamcode.common.util.geometry.Polygon
import kotlin.math.abs

@Configurable
object ZoneAssist {
    @JvmField
    var wallClearance = 18.0 / 2.0 // how much to shrink the zones near walls to account for the size of the bot

    @JvmField
    var botRadius = 5.0

    val closeZone
        get() = Polygon.autoConvex(
            Vec2d(14, 128) + Vec2d(wallClearance, -wallClearance),
            Vec2d(22, 144) + Vec2d(wallClearance, -wallClearance),
            Vec2d(120, 144) + Vec2d(-wallClearance, -wallClearance),
            Vec2d(130, 128) + Vec2d(-wallClearance, -wallClearance),
            Vec2d(72, 72),
        )

    val closeZoneFull
        get() = Polygon.autoConvex(
            Vec2d(0, 144) + Vec2d(botRadius, 0).rotate((90.0).toDegrees()),
            Vec2d(0, 144) + Vec2d(botRadius, 0).rotate((-180.0).toDegrees()),
            Vec2d(0, 144) + Vec2d(botRadius, 0).rotate((-135.0).toDegrees()),
            Vec2d(72, 72) + Vec2d(botRadius, 0).rotate((-135.0).toDegrees()),
            Vec2d(72, 72) + Vec2d(botRadius, 0).rotate((-90.0).toDegrees()),
            Vec2d(72, 72) + Vec2d(botRadius, 0).rotate((-45.0).toDegrees()),
            Vec2d(144, 144) + Vec2d(botRadius, 0).rotate((-45.0).toDegrees()),
            Vec2d(144, 144) + Vec2d(botRadius, 0).rotate((0.0).toDegrees()),
            Vec2d(144, 144) + Vec2d(botRadius, 0).rotate((90.0).toDegrees()),
        )

    val farZone
        get() = Polygon.autoConvex(
            Vec2d(48, 0) + Vec2d(wallClearance, wallClearance),
            Vec2d(96, 0) + Vec2d(-wallClearance, wallClearance),
            Vec2d(72, 24),
        )

    val farZoneFull
        get() = Polygon.autoConvex(
            Vec2d(48, 0) + Vec2d(botRadius, 0).rotate((-90.0).toDegrees()),
            Vec2d(96, 0) + Vec2d(botRadius, 0).rotate((-90.0).toDegrees()),
            Vec2d(96, 0) + Vec2d(botRadius, 0).rotate((0.0).toDegrees()),
            Vec2d(96, 0) + Vec2d(botRadius, 0).rotate((45.0).toDegrees()),
            Vec2d(72, 24) + Vec2d(botRadius, 0).rotate((45.0).toDegrees()),
            Vec2d(72, 24) + Vec2d(botRadius, 0).rotate((90.0).toDegrees()),
            Vec2d(72, 24) + Vec2d(botRadius, 0).rotate((135.0).toDegrees()),
            Vec2d(48, 0) + Vec2d(botRadius, 0).rotate((180.0).toDegrees()),
        )

    @JvmField
    var strength = 0.15

    @JvmField
    var curve = 4.0 / 5.0

    @JvmField
    var epilsonBase = 2.0

    @JvmField
    var epilsonGain = 1.0

    fun inZonePartial(pos: Vec2d): Boolean = closeZoneFull.contains(pos) || farZoneFull.contains(pos)

    fun inZone(pos: Vec2d): Boolean = closeZone.contains(pos) || farZone.contains(pos)

    fun calculateAssist(pos: Vec2d): Vec2d {
        if (inZone(pos)) {
            return Vec2d(0.0, 0.0)
        }

        val closeZone = this.closeZone
        val farZone = this.farZone

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