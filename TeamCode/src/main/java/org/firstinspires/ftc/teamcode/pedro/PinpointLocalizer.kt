package org.firstinspires.ftc.teamcode.pedro

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.util.Pose2d
import com.pedropathing.ftc.FollowerBuilder
import com.pedropathing.geometry.Pose
import com.pedropathing.localization.Localizer
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.common.hardware.fromFTC
import org.firstinspires.ftc.teamcode.common.hardware.fromPedro
import org.firstinspires.ftc.teamcode.common.hardware.toFTC
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.common.subsystem.Limelight
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min


class FusionLocalizer(hardwareMap: HardwareMap, val deltaTime: () -> Double, val limelight: Limelight? = null) : Localizer {
    val pinpoint = (hardwareMap.get("pinpoint") as GoBildaPinpointDriver).apply {
        setOffsets(PinpointSettings.forwardOffset, PinpointSettings.strafeOffset, DistanceUnit.MM)
        setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)

        val reversedPod = GoBildaPinpointDriver.EncoderDirection.REVERSED
        val forwardPod = GoBildaPinpointDriver.EncoderDirection.FORWARD
        setEncoderDirections(
            if (PinpointSettings.forwardReversed) reversedPod else forwardPod,
            if (PinpointSettings.strafeReversed) reversedPod else forwardPod
        )

        resetPosAndIMU()
    }

    var _pose: Pose2d = Pose2d()
    var _drift: Pose2d = Pose2d()
    var _velocity: Pose2d = Pose2d()

    var _lastLLTimestamp = 0L

    val kfX = DriftKalmanFilter()
    val kfY = DriftKalmanFilter()

    override fun getPose() = (_pose - _drift).toPedro()
    override fun getVelocity() = _velocity.toPedro()
    override fun getVelocityVector() = _velocity.toPedro().asVector!!

    override fun setStartPose(p0: Pose) = setPose(p0) // low-key cannot be bothered to compensate, just don't update?

    override fun setPose(p0: Pose) {
        pinpoint.position = Pose2d.fromPedro(p0).toFTC()
        kfX.reset()
        kfY.reset()
    }

    override fun update() {
        pinpoint.update()
        val pose = Pose2d.fromFTC(pinpoint.position)
        _pose = pose

        kfX.predict(deltaTime())
        kfY.predict(deltaTime())

        if (limelight != null) {
            val llPose = limelight.pose
            if (llPose != null && llPose.second != _lastLLTimestamp) {
                _lastLLTimestamp = llPose.second

                kfX.update(pose.x, llPose.first.x)
                kfY.update(pose.y, llPose.first.y)
            }
        }

        _drift = Pose2d(kfX.drift, kfY.drift, 0.0)

        _velocity = Pose2d(
            pinpoint.getVelX(DistanceUnit.INCH),
            pinpoint.getVelY(DistanceUnit.INCH),
            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)
        )
    }

    override fun getTotalHeading() = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)

    override fun getForwardMultiplier() = pinpoint.encoderY.toDouble()
    override fun getLateralMultiplier() = pinpoint.encoderX.toDouble()
    override fun getTurningMultiplier() = pinpoint.yawScalar.toDouble()

    override fun resetIMU() = pinpoint.resetPosAndIMU()
    override fun getIMUHeading(): Double = Double.NaN
    override fun isNAN(): Boolean = pose.x.isNaN() || pose.y.isNaN() || pose.heading.isNaN()
}

fun FollowerBuilder.fusionLocalizer(hardwareMap: HardwareMap, deltaTime: () -> Double, limelight: Limelight? = null): FollowerBuilder {
    return setLocalizer(FusionLocalizer(hardwareMap, deltaTime, limelight))
}

@Configurable
class DriftKalmanFilter() {
    var drift = 0.0
    var uncertainty = startingUncertainty

    fun reset() {
        drift = 0.0
        uncertainty = startingUncertainty
    }

    fun predict(dt: Double) {
        uncertainty += processNoise * dt
    }

    fun update(pinpoint: Double, limelight: Double) {
        val rawDrift = pinpoint - limelight
        if (abs(rawDrift - drift) > maxDist) return

        val rawGain = uncertainty / (uncertainty + measurementNoise)
        val gain = min(maxGain * (1 - exp(-rawGain / maxGain)), maxGain)

        drift += gain * (rawDrift - drift)

        uncertainty = max(uncertainty * (1 - gain), minUncertainty)
    }

    companion object {
        @JvmField
        var startingUncertainty = 1.0

        @JvmField
        var processNoise = 0.01 // q

        @JvmField
        var measurementNoise = 1.0 // r

        @JvmField
        var maxGain = 0.3

        @JvmField
        var maxDist = 12.0

        @JvmField
        var minUncertainty = 1e-6
    }
}

@Configurable
object PinpointSettings {
    @JvmField
    var forwardOffset = -150.0

    @JvmField
    var strafeOffset = -138.0

    @JvmField
    var forwardReversed = false

    @JvmField
    var strafeReversed = false
}