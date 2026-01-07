package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Configurable
class Intake(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Intake") {
    val frontMotor = ManualMotor(opMode.hardwareMap, frontMotorName, reverse = frontMotorReversed, float = true)

    var frontPower = 0.0

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    val power = gp1.current.rightTrigger - gp1.current.leftTrigger
                    frontPower = power
                }
                frontMotor.power = frontPower
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var frontMotorName = "m3e"

        @JvmField
        var frontMotorReversed = true
    }
}