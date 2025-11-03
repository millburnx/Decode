package org.firstinspires.ftc.teamcode.pedro

import com.pedropathing.follower.Follower
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.ftc.FollowerBuilder
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.ftc.localization.constants.PinpointConstants
import com.pedropathing.paths.PathConstraints
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.PI

object Constants {
    val followerConstants: FollowerConstants = FollowerConstants()
        .mass(10.0)
        .forwardZeroPowerAcceleration(-154.736)
        .lateralZeroPowerAcceleration(-148.621)
        .centripetalScaling(0.0)
//        .translationalPIDFCoefficients(PIDFCoefficients(0.0, 0.0, 0.0, 0.0))
//        .headingPIDFCoefficients(PIDFCoefficients(0.0, 0.0, 0.0, 0.0))
//        .drivePIDFCoefficients(FilteredPIDFCoefficients(0.0, 0.0, 0.0, 0.6, 0.0))
//        .useSecondaryTranslationalPIDF(true)
//        .useSecondaryHeadingPIDF(true)
//        .useSecondaryDrivePIDF(true)
//        .secondaryTranslationalPIDFCoefficients(PIDFCoefficients(0.0, 0.0, 0.0, 0.0))
//        .secondaryHeadingPIDFCoefficients(PIDFCoefficients(0.0, 0.0, 0.0, 0.0))
//        .secondaryDrivePIDFCoefficients(FilteredPIDFCoefficients(0.0, 0.0, 0.0, 0.0, 0.0))

    fun MecanumConstants.setMotors() = apply {
        rightFrontMotorName("m2")
        rightRearMotorName("m3")
        leftFrontMotorName("m0")
        leftRearMotorName("m1")
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
        .customEncoderResolution(OpenOdo.ENCODER_RESOLUTION)
        .forwardPodY(-6.375)
        .strafePodX(1.44)
        .forwardEncoderDirection(EncoderDirection.REVERSED)
        .strafeEncoderDirection(EncoderDirection.REVERSED)

    val pathConstraints: PathConstraints = PathConstraints(0.99, 100.0, 1.0, 1.0)

    fun createFollower(hardwareMap: HardwareMap): Follower {
        return FollowerBuilder(followerConstants, hardwareMap)
            .mecanumDrivetrain(driveConstants)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .build()
    }
}


object OpenOdo {
    const val TICKS_PER_REVOLUTION = 8192.0
    const val DIAMETER_MM = 35.0
    const val ENCODER_RESOLUTION_MM = TICKS_PER_REVOLUTION / (DIAMETER_MM * PI)
    const val ENCODER_RESOLUTION = ENCODER_RESOLUTION_MM / 25.4
};