package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.millburnx.cmdx.Command
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Pose2d

class Odom(opMode: LinearOpMode, tel: MultipleTelemetry) : Subsystem("Odom") {
    val odom = opMode.hardwareMap["odom"] as GoBildaPinpointDriver

    init {
        val diameterMM = 35.0
        val ticksPerRevolution = 8192.0
        val circumferenceMM = diameterMM * Math.PI
        val ticksPerMM = ticksPerRevolution / circumferenceMM
        odom.setEncoderResolution(ticksPerMM, DistanceUnit.MM)
        odom.setOffsets(5.75, -0.75, DistanceUnit.INCH)
        odom.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
        )
        odom.resetPosAndIMU()
    }

    val pose: Pose2d
        get() = Pose2d(
            odom.position.getX(DistanceUnit.INCH),
            odom.position.getY(DistanceUnit.INCH),
            odom.position.getHeading(AngleUnit.DEGREES)
        )
    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            while (opModeIsActive() && !isStopRequested) {
                odom.update()

                tel.addData("Pose", pose)
            }
        }
    }

    override val command = Command(this.name, cleanup, run)
}