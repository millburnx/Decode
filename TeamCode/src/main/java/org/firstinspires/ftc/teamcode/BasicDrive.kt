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
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.ManualManager
import org.firstinspires.ftc.teamcode.util.Pose2d
import org.firstinspires.ftc.teamcode.util.Vec2d
import kotlin.math.abs
import kotlin.math.max


@TeleOp(name = "BasicDrive")
class BasicDrive : LinearOpMode() {
    val timer = ElapsedTime()
    val scheduler = CommandScheduler().apply {
        onSync = {
            val loopHertz = 1.0 / timer.seconds()
            timer.reset()

            telemetry.addData("hz", loopHertz)
            telemetry.update()

            hubs.forEach { it.clearBulkCache() }
            ManualManager.update()
        }
    }

    var hubs: List<LynxModule> = emptyList()

    override fun runOpMode() {
        hubs = hardwareMap.getAll(LynxModule::class.java) as MutableList<LynxModule>
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO }

        val br = hardwareMap["m3"] as DcMotorEx // m3
        val fl = hardwareMap["m0"] as DcMotorEx // m0
        val bl = hardwareMap["m1"] as DcMotorEx // m1
        val fr = hardwareMap["m2"] as DcMotorEx // m2

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
                    val strafe = -gamepad1.left_stick_x.toDouble()
                    val rotate = gamepad1.right_stick_x.toDouble()


                    val heading = odom.getHeading(AngleUnit.RADIANS)

                    val rVec = Vec2d(strafe, forward).rotate(heading - Math.toRadians(90.0)) * Vec2d(1.0, 1.1)

                    val denominator = max(abs(rVec.x) + abs(rVec.y) + abs(rotate), 1.0)

//                    val right_y = gamepad1.right_stick_y.toDouble()

//                    fl.power = forward;
//                    fr.power = strafe;
//                    br.power = rotate;
//                    bl.power = right_y;

                    fl.power = (rVec.x + rVec.y + rotate) / denominator
                    fr.power = (rVec.x - rVec.y - rotate) / denominator
                    br.power = (rVec.x + rVec.y - rotate) / denominator
                    bl.power = (rVec.x - rVec.y + rotate) / denominator

                    sync()
                }
            },
        )

        scheduler.schedule(
            Command {
                while (opModeIsActive() && !isStopRequested) {
                    odom.update()

                    val pose = odom.position
                    val pose2D = Pose2d(
                        pose.getX(DistanceUnit.INCH),
                        pose.getY(DistanceUnit.INCH),
                        pose.getHeading(AngleUnit.DEGREES)
                    )

                    telemetry.addData("pose", pose2D.toString())

                    sync()
                }
            }
        )

        while (opModeIsActive() && !isStopRequested) {
        }
    }
}
