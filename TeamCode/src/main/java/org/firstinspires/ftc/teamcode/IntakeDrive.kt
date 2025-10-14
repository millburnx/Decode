package org.firstinspires.ftc.teamcode

import com.millburnx.cmdx.Command
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.motorSetup
import kotlin.math.abs
import kotlin.math.max


@TeleOp(name = "IntakeDrive")
class IntakeDrive : LinearOpMode() {
    val scheduler = CommandScheduler()

    override fun runOpMode() {
        val hubs = hardwareMap.getAll(LynxModule::class.java) as MutableList<LynxModule>
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO }

        val fl = (hardwareMap["m0"] as DcMotorEx).apply { motorSetup(this, reverse = true) }
        val fr = (hardwareMap["m3"] as DcMotorEx).apply { motorSetup(this, reverse = false) }
        val br = (hardwareMap["m2"] as DcMotorEx).apply { motorSetup(this, reverse = false) }
        val bl = (hardwareMap["m1"] as DcMotorEx).apply { motorSetup(this, reverse = true) }

        val intake = Intake(this)
        val flyWheel = FlyWheel(this)

        telemetry.isAutoClear = true

        waitForStart()

        scheduler.schedule(intake.command)
        scheduler.schedule(flyWheel.command)

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

//                    sync()
                }
            },
        )

        while (opModeIsActive() && !isStopRequested) {
//            println("Job Debug, commands: ${scheduler.runner.commands.map { it.name }}, channels: ${scheduler.runner.channels.map { it.component2() }}")
        }
    }
}
