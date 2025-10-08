package org.firstinspires.ftc.teamcode

import com.millburnx.cmdx.Command
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple


@TeleOp(name = "MotorTest")
class MotorTest : LinearOpMode() {
    val scheduler = CommandScheduler()

    override fun runOpMode() {
        val hubs = hardwareMap.getAll(LynxModule::class.java) as MutableList<LynxModule>
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO }

        val motor = hardwareMap["m0"] as DcMotorEx
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
        motor.direction = DcMotorSimple.Direction.FORWARD
        telemetry.isAutoClear = true

        waitForStart()

        scheduler.schedule(
            Command {
                while (opModeIsActive() && !isStopRequested) {
                    val power = gamepad1.left_stick_x
                    motor.power = power.toDouble()
                    telemetry.addData("Power: ", power.toString())
                    telemetry.update()

                    sync()
                }
            },
        )

        while (opModeIsActive() && !isStopRequested) {
        }
    }
}
