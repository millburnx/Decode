package org.firstinspires.ftc.teamcode.pedro

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
import org.firstinspires.ftc.teamcode.common.subsystem.Limelight

object Constants {
    val followerConstants: FollowerConstants = FollowerConstants()
        .mass(12.2)
        .forwardZeroPowerAcceleration(-68.384)
        .lateralZeroPowerAcceleration(-96.985)
        .centripetalScaling(0.0)
        .translationalPIDFCoefficients(PIDFCoefficients(0.2, 0.0, 0.02, 0.03))
        .headingPIDFCoefficients(PIDFCoefficients(3.0, 0.0, 0.1, 0.04))
        .drivePIDFCoefficients(FilteredPIDFCoefficients(0.03, 0.0, 0.0, 0.6, 0.05))
        .useSecondaryTranslationalPIDF(true)
        .useSecondaryHeadingPIDF(true)
        .useSecondaryDrivePIDF(false)
        .secondaryTranslationalPIDFCoefficients(PIDFCoefficients(0.5, 0.0, 0.03, 0.03))
        .secondaryHeadingPIDFCoefficients(PIDFCoefficients(4.0, 0.0, 0.2, 0.04))
        .secondaryDrivePIDFCoefficients(FilteredPIDFCoefficients(0.015, 0.0, 0.0, 0.6, 0.3))

    fun MecanumConstants.setMotors() = apply {
        rightFrontMotorName("m2")
        rightRearMotorName("m0")
        leftFrontMotorName("m1")
        leftRearMotorName("m3")
        rightFrontMotorDirection(Direction.REVERSE)
        rightRearMotorDirection(Direction.REVERSE)
        leftFrontMotorDirection(Direction.FORWARD)
        leftRearMotorDirection(Direction.FORWARD)
    }

    fun MecanumConstants.setPower() = apply {
        maxPower(1.0)
        xVelocity(60.098)
        yVelocity(48.711)
    }

    val driveConstants: MecanumConstants = MecanumConstants()
        .setMotors()
        .setPower()

    val localizerConstants: PinpointConstants = PinpointConstants()
        .hardwareMapName("pinpoint")
        .distanceUnit(DistanceUnit.INCH)
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
        .forwardPodY(-5.28401)
        .strafePodX(-0.15106)
        .forwardEncoderDirection(EncoderDirection.FORWARD)
        .strafeEncoderDirection(EncoderDirection.REVERSED)

    val pathConstraints: PathConstraints = PathConstraints(0.99, 100.0, .5, 2.0)

    fun createFollower(hardwareMap: HardwareMap): Follower {
        return FollowerBuilder(followerConstants, hardwareMap)
            .mecanumDrivetrain(driveConstants)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .build()
    }

    fun createManualFollower(hardwareMap: HardwareMap): Follower {
        return FollowerBuilder(followerConstants, hardwareMap)
            .manualMecanumDrivetrain(hardwareMap, driveConstants)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .build()
    }

    fun createManualFusionFollower(hardwareMap: HardwareMap, deltaTime: () -> Double, limelight: Limelight? = null): Follower {
        return FollowerBuilder(followerConstants, hardwareMap)
            .manualMecanumDrivetrain(hardwareMap, driveConstants)
            .fusionLocalizer(hardwareMap, deltaTime, limelight)
            .pathConstraints(pathConstraints)
            .build()
    }
}