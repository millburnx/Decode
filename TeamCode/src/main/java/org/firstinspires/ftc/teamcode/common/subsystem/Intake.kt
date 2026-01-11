package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Configurable
class Intake(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Intake") {
    val motor = ManualMotor(opMode.hardwareMap, motorName, reverse = motorReversed, float = true).motor

    var power = 0.0

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    val power = gp1.current.rightTrigger - gp1.current.leftTrigger
                    this@Intake.power = power
                }
                motor.power = power
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var motorName = "m3e"

        @JvmField
        var motorReversed = true
    }
}