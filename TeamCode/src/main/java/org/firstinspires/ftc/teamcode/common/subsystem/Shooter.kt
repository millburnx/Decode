package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode

class Shooter(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Shooter") {
    val motor = ManualMotor(opMode.hardwareMap, motorName, reverse = motorReversed, float = true)

    var power = 0.0

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    power = gp2.current.rightTrigger - gp2.current.leftTrigger
                }
                motor.power = power
                tel.addData("shooter | power", power)
                tel.addData("shooter | rpm", motor.velocity * 60.0 / 28.0) // 28 ppr @ 6k rpm
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var motorName = "m3e"

        @JvmField
        var motorReversed = false
    }
}