package org.firstinspires.ftc.teamcode.Pedro;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .forwardZeroPowerAcceleration(-154.736)
            .lateralZeroPowerAcceleration(-148.621)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.002, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(3.5, 0, 0.28, 0.01));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("m2")
            .rightRearMotorName("m3")
            .leftFrontMotorName("m0")
            .leftRearMotorName("m1")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(140) // 147.58413
            .yVelocity(131.245);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.3, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("odom")
            .distanceUnit(DistanceUnit.INCH)
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .customEncoderResolution((74.505/17.8)*1.5*0.55*0.86*0.9375) // increasing this value decreases x: reading
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .forwardPodY(-6.375)
            .strafePodX(1.44);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}