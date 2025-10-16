package org.firstinspires.ftc.teamcode.subsystems

import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.ManualMotor
import kotlin.math.abs
import kotlin.math.max

class Drivetrain(opMode: LinearOpMode) : Subsystem("Intake") {
    val fl = ManualMotor(opMode.hardwareMap, "m0", reverse = false)
    val fr = ManualMotor(opMode.hardwareMap, "m1", reverse = true)
    val br = ManualMotor(opMode.hardwareMap, "m2", reverse = true)
    val bl = ManualMotor(opMode.hardwareMap, "m3", reverse = false)

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            while (opModeIsActive() && !isStopRequested) {
                val y = -gamepad1.left_stick_y.toDouble()
                val x = gamepad1.left_stick_x * 1.1
                val rx = -gamepad1.right_stick_x.toDouble()

                val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)
                fl.power = (y + x + rx) / denominator
                fr.power = (y - x - rx) / denominator
                br.power = (y + x - rx) / denominator
                bl.power = (y - x + rx) / denominator

                sync()
            }
        }
    }

    override val command = Command(this.name, cleanup, run)
}