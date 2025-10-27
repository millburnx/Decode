package org.firstinspires.ftc.teamcode.subsystems

import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.ManualMotor
import org.firstinspires.ftc.teamcode.util.Pose2d
import kotlin.math.abs
import kotlin.math.max

class Drivetrain(opMode: LinearOpMode) : Subsystem("Intake") {
    val fl = ManualMotor(opMode.hardwareMap, "m0", reverse = true)
    val fr = ManualMotor(opMode.hardwareMap, "m1", reverse = false)
    val br = ManualMotor(opMode.hardwareMap, "m2", reverse = false)
    val bl = ManualMotor(opMode.hardwareMap, "m3", reverse = true)

    fun drive(direction: Pose2d) {
        val denominator = max(abs(direction.y) + abs(direction.x) + abs(direction.heading), 1.0)
        fl.power = (direction.y + direction.x + direction.heading) / denominator
        fr.power = (direction.y - direction.x - direction.heading) / denominator
        br.power = (direction.y + direction.x - direction.heading) / denominator
        bl.power = (direction.y - direction.x + direction.heading) / denominator
    }
    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            while (opModeIsActive() && !isStopRequested) {
                val y = -gamepad1.left_stick_y.toDouble()
                val x = gamepad1.left_stick_x * 1.1
                val rx = gamepad1.right_stick_x.toDouble()

//                val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)
//                fl.power = (y + x + rx) / denominator
//                fr.power = (y - x - rx) / denominator
//                br.power = (y + x - rx) / denominator
//                bl.power = (y - x + rx) / denominator

                drive(Pose2d(x,y,rx))

                sync()
            }
        }
    }

    override val command = Command(this.name, cleanup, run)
}