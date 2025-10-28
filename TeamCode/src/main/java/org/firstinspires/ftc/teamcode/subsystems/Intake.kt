package org.firstinspires.ftc.teamcode.subsystems

import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.ManualMotor

class Intake(opMode: LinearOpMode) : Subsystem("Intake") {
    val motor = ManualMotor(opMode.hardwareMap, "m2e", reverse = true)
    var power: Double = 0.0
    var locked: Boolean = false

    override val run: suspend Command.() -> Unit = {
        with (opMode) {
            while (opModeIsActive() && !isStopRequested) {
                if (!locked) {
                    power = -(gamepad1.right_trigger - gamepad1.left_trigger).toDouble()
                }

                motor.power = power
                sync()
            }
        }
    }

    override val command = Command(this.name,cleanup,run)
}