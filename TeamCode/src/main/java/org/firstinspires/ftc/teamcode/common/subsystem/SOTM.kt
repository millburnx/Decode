package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.millburnx.util.Pose2d
import com.millburnx.util.vector.Vec2d

class SOTM {
    fun calculate(velocity: Vec2d, pose: Pose2d) {
        val rawGoal = Vec2d(140.0, 140.0)

        val rawAngle = pose.position.angleTo(rawGoal)

        val rotatedVelocity = velocity.rotate(rawAngle)
        val forwardVelocity = rotatedVelocity.y
        val lateralVelocity = rotatedVelocity.x

        val scaledVelocity = Vec2d(
            (forwardVelocity * forwardScale).clamp(-maxForwardScale, maxForwardScale),
            (lateralVelocity * lateralScale).clamp(-maxLateralScale, maxLateralScale)
        ).rotate(-rawAngle)

        val newGoal = pose.position - scaledVelocity

        val distance = pose.distanceTo(newGoal)
        val targetAngle = pose.position.angleTo(newGoal)
    }

    companion object {
        @JvmField
        var forwardScale = 1.0

        @JvmField
        var lateralScale = 1.0

        @JvmField
        var maxForwardScale = 1.0

        @JvmField
        var maxLateralScale = 1.0
    }
}