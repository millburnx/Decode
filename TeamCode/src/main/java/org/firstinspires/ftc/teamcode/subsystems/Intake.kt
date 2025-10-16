package org.firstinspires.ftc.teamcode.subsystems

import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx

class Intake(opMode: LinearOpMode) : Subsystem("Intake") {
    val motor = (opMode.hardwareMap["m0e"] as DcMotorEx).apply { motorSetup(this, reverse = true) }
    var power: Double = 0.0

    override val run: suspend Command.() -> Unit = {
        with (opMode) {
            while (opModeIsActive() && !isStopRequested) {
                power = -(gamepad1.right_trigger - gamepad1.left_trigger).toDouble()

                motor.power = power
                sync()
            }
        }
    }

    override val command = Command(this.name,cleanup,run)
}