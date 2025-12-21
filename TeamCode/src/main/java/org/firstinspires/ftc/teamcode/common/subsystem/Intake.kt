package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode


class Intake(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Intake") {
    val frontMotor = ManualMotor(opMode.hardwareMap, frontMotorName, reverse = frontMotorReversed, float = true)
    val backMotor = ManualMotor(opMode.hardwareMap, backMotorName, reverse = backMotorReversed, float = true)

    var frontPower = 0.0
    var backPower = 0.0

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    val power = gp1.current.rightTrigger - gp1.current.leftTrigger
                    frontPower = power
                    backPower = power
                }
                frontMotor.power = frontPower
                backMotor.power = backPower
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var frontMotorName = "m0e"

        @JvmField
        var backMotorName = "m1e"

        @JvmField
        var frontMotorReversed = false

        @JvmField
        var backMotorReversed = false
    }
}