package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode

class Intake(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Intake") {
    val motor = ManualMotor(opMode.hardwareMap, "m3e")
    var power = 0.0

    override val run: suspend Command.() -> Unit = {
        with (opMode) {
            WaitFor { isStarted || !isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    power = (gp1.right_trigger - gp1.left_trigger).toDouble();
                }
                motor.power = power
                sync()
            }
        }
    }
}