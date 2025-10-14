package org.firstinspires.ftc.teamcode

import com.millburnx.cmdx.Command
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs
import kotlin.math.max


@TeleOp(name = "BasicDrive")
class BasicDrive : LinearOpMode() {
    val scheduler = CommandScheduler()

    override fun runOpMode() {
        val hubs = hardwareMap.getAll(LynxModule::class.java) as MutableList<LynxModule>
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO }

        val fl = hardwareMap["m0"] as DcMotorEx
        val fr = hardwareMap["m3"] as DcMotorEx
        val br = hardwareMap["m2"] as DcMotorEx
        val bl = hardwareMap["m1"] as DcMotorEx

        val odom = hardwareMap["odom"] as GoBildaPinpointDriver

        fl.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        fr.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        br.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        bl.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        fl.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
        fr.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
        br.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
        bl.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS

        fl.direction = DcMotorSimple.Direction.REVERSE
        fr.direction = DcMotorSimple.Direction.FORWARD
        br.direction = DcMotorSimple.Direction.FORWARD
        bl.direction = DcMotorSimple.Direction.REVERSE


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

        telemetry.isAutoClear = true

        waitForStart()

        scheduler.schedule(
            Command {
                while (opModeIsActive() && !isStopRequested) {
                    val forward = -gamepad1.left_stick_y.toDouble()
                    val strafe = gamepad1.left_stick_x.toDouble()
                    val rotate = gamepad1.right_stick_x.toDouble()

                    val heading = odom.getHeading(AngleUnit.RADIANS)

                    val rVec = Vec2d(forward, strafe).rotate(heading - Math.toRadians(90.0)) * Vec2d(1.0, 1.1)

                    val denominator = max(abs(rVec.x) + abs(rVec.y) + abs(rotate), 1.0)
                    fl.power = (rVec.x + rVec.y + rotate) / denominator
                    fr.power = (rVec.x - rVec.y - rotate) / denominator
                    br.power = (rVec.x + rVec.y - rotate) / denominator
                    bl.power = (rVec.x - rVec.y + rotate) / denominator

//                    sync()
                }
            },
        )

        scheduler.schedule(
            Command {
                while (opModeIsActive() && !isStopRequested) {
                    odom.update()

//                    sync()
                }
            }
        )

        while (opModeIsActive() && !isStopRequested) {
         }
    }
}
