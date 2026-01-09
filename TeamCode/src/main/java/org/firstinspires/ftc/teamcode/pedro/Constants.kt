package org.firstinspires.ftc.teamcode.pedro

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.control.FilteredPIDFCoefficients
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.follower.Follower
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.ftc.FollowerBuilder
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.ftc.localization.constants.PinpointConstants
import com.pedropathing.paths.PathConstraints
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.common.hardware.manual.manualMecanumDrivetrain

object Constants {
    val followerConstants: FollowerConstants = FollowerConstants()
        .mass(12.2)
        .forwardZeroPowerAcceleration(-154.736)
        .lateralZeroPowerAcceleration(-148.621)
        .centripetalScaling(0.0)
        .translationalPIDFCoefficients(PIDFCoefficients(0.35, 0.0, 0.01, 0.03))
        .headingPIDFCoefficients(PIDFCoefficients(1.5, 0.0, 0.1, 0.03))
        .drivePIDFCoefficients(FilteredPIDFCoefficients(0.01, 0.0, 0.0, 0.6, 0.3))
        .useSecondaryTranslationalPIDF(true)
        .useSecondaryHeadingPIDF(true)
        .useSecondaryDrivePIDF(true)
        .secondaryTranslationalPIDFCoefficients(PIDFCoefficients(0.4, 0.0, 0.03, 0.03))
        .secondaryHeadingPIDFCoefficients(PIDFCoefficients(2.0, 0.0, 0.2, 0.03))
        .secondaryDrivePIDFCoefficients(FilteredPIDFCoefficients(0.015, 0.0, 0.0, 0.6, 0.3))

    fun MecanumConstants.setMotors() = apply {
        rightFrontMotorName("m1")
        rightRearMotorName("m3")
        leftFrontMotorName("m0")
        leftRearMotorName("m2")
        leftFrontMotorDirection(Direction.REVERSE)
        leftRearMotorDirection(Direction.REVERSE)
        rightFrontMotorDirection(Direction.FORWARD)
        rightRearMotorDirection(Direction.FORWARD)
    }

    fun MecanumConstants.setPower() = apply {
        maxPower(1.0)
        xVelocity(147.58413)
        yVelocity(131.245)
    }

    val driveConstants: MecanumConstants = MecanumConstants()
        .setMotors()
        .setPower()

    val localizerConstants: PinpointConstants = PinpointConstants()
        .hardwareMapName("pinpoint")
        .distanceUnit(DistanceUnit.INCH)
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
        .forwardPodY(5.28401)
        .strafePodX(-0.15106)
        .forwardEncoderDirection(EncoderDirection.FORWARD)
        .strafeEncoderDirection(EncoderDirection.REVERSED)

    val pathConstraints: PathConstraints = PathConstraints(0.99, 100.0, .25, 2.0)

    fun createFollower(hardwareMap: HardwareMap): Follower {
        return FollowerBuilder(followerConstants, hardwareMap)
            .manualMecanumDrivetrain(hardwareMap, driveConstants)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .build()
    }
}

@Configurable
object Odom {

};