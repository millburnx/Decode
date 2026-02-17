package org.firstinspires.ftc.teamcode.pedro

import com.millburnx.util.Pose2d
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

class FusionLocalizer(hardwareMap: HardwareMap, val limelight: Limelight?) : Localizer {
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
    var _velocity: Pose2d = Pose2d()
//    var _totalHeading: Double = 0.0

    override fun getPose() = _pose.toPedro()
    override fun getVelocity() = _velocity.toPedro()
    override fun getVelocityVector() = _velocity.toPedro().asVector!!

    override fun setStartPose(p0: Pose) = setPose(p0) // low-key cannot be bothered to compensate, just don't update?

    override fun setPose(p0: Pose) {
        pinpoint.position = Pose2d.fromPedro(p0).toFTC()
    }

    override fun update() {
        pinpoint.update()
        val pose = Pose2d.fromFTC(pinpoint.position)
//        _totalHeading += normalizeDegrees(pose.degrees - _pose.heading)

        _pose = pose
        _velocity = Pose2d(
            pinpoint.getVelX(DistanceUnit.INCH),
            pinpoint.getVelY(DistanceUnit.INCH),
            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)
        )
    }

    override fun getTotalHeading() = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES)

    override fun getForwardMultiplier() = pinpoint.encoderY.toDouble()
    override fun getLateralMultiplier() = pinpoint.encoderX.toDouble()
    override fun getTurningMultiplier() = pinpoint.yawScalar.toDouble()

    override fun resetIMU() = pinpoint.resetPosAndIMU()
    override fun getIMUHeading(): Double = Double.NaN
    override fun isNAN(): Boolean = pose.x.isNaN() || pose.y.isNaN() || pose.heading.isNaN()
}

object PinpointSettings {
    @JvmField
    var forwardOffset = -150.0

    @JvmField
    var strafeOffset = -125.0

    @JvmField
    var forwardReversed = false

    @JvmField
    var strafeReversed = false
}
