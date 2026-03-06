package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class Intake(opMode: OpMode) : Subsystem("Intake") {
    val motor = ManualMotor(opMode.hardwareMap, motorName, motorReversed)

    var power = 0.0

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            motor.power = power
        }
    }



    companion object {
        @JvmField
        var motorName = "m1e"

        @JvmField
        var motorReversed = true
    }
}