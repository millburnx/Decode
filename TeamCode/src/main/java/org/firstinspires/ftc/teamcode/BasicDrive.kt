package org.firstinspires.ftc.teamcode

import com.millburnx.cmdx.Command
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.max

@TeleOp(name = "BasicDrive")
class BasicDrive : LinearOpMode() {
    val scheduler = CommandScheduler()

    override fun runOpMode() {
        val fl = hardwareMap["m0"] as DcMotorEx
        val fr = hardwareMap["m3"] as DcMotorEx
        val br = hardwareMap["m2"] as DcMotorEx
        val bl = hardwareMap["m1"] as DcMotorEx

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

        waitForStart()

        scheduler.schedule(
            Command {
                while (opModeIsActive() && !isStopRequested) {
                    val y = -gamepad1.left_stick_y.toDouble()
                    val x = gamepad1.left_stick_x * 1.1
                    val rx = gamepad1.right_stick_x.toDouble()

                    val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)
                    fl.power = (y + x + rx) / denominator
                    fr.power = (y - x - rx) / denominator
                    br.power = (y + x - rx) / denominator
                    bl.power = (y - x + rx) / denominator

                    sync()
                }
            },
        )


        while (opModeIsActive() && !isStopRequested) {}
    }
}
