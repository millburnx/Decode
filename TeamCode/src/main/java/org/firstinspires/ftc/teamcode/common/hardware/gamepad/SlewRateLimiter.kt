package org.firstinspires.ftc.teamcode.common.hardware.gamepad

import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d
import com.millburnx.cmdxpedro.util.toDegrees
import org.firstinspires.ftc.teamcode.common.normalizeDegrees
import kotlin.math.abs

object SlewRateLimiter {
    fun limit(current: Double, target: Double, maxRate: Double): Double {
        val rate = target - current
        return current + rate.coerceIn(-maxRate, maxRate)
    }

    fun limit(current: Vec2d, target: Vec2d, maxMagnitudeRate: Double, maxRotationRate: Double): Vec2d {
        // if angle diff < 90, slew angle and magnitude, if angle > thresh, focus on slewing magnitude (cross-over!)
        val targetAngle = Vec2d().angleTo(target).toDegrees()
        val currentAngle = Vec2d().angleTo(current).toDegrees()
        val angleDiff = normalizeDegrees(targetAngle - currentAngle)

        if (abs(angleDiff) <= 135) {
            // TODO: Implement this stuff
        }

        return Vec2d()
    }
}