package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.TelemetryManager
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees

@Configurable
object SOTM {
    fun calculate(tel: TelemetryManager, velocity: Vec2d, pose: Pose2d, rawGoal: Vec2d): Pair<Double, Double> {
        val rawAngle = pose.position.angleTo(rawGoal)
        val rawDistance = pose.distanceTo(rawGoal)

        val rotatedVelocity = velocity.rotate(-rawAngle)
        val forwardVelocity = rotatedVelocity.x
        val lateralVelocity = rotatedVelocity.y

        val scaledVelocity = Vec2d(
            forwardVelocity * (rawDistance * forwardScale).clamp(-maxForwardScale, maxForwardScale),
            lateralVelocity * (rawDistance * lateralScale).clamp(-maxLateralScale, maxLateralScale)
        ).rotate(rawAngle)

        val newGoal = rawGoal - scaledVelocity

        val distance = pose.distanceTo(newGoal)
        val targetAngle = pose.position.angleTo(newGoal).toDegrees()

        tel.addData("angle diff", normalizeDegrees(targetAngle - rawAngle.toDegrees()))
        tel.addData("dist diff", distance - rawDistance)

        return distance to targetAngle
    }

    @JvmField
    var forwardScale = 0.001

    @JvmField
    var lateralScale = 0.01

    @JvmField
    var maxForwardScale = 1.0

    @JvmField
    var maxLateralScale = 1.0
}