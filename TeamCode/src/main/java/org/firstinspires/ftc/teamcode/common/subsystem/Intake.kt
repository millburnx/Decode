package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.sign

@Configurable
class Intake(opMode: OpMode) : Subsystem("Intake") {
    val motor = ManualMotor(opMode.hardwareMap, motorName, motorReversed)

    private var _power = 0.0

    var power
        get() = _power
        set(value) {
            _power = value
            targetPower = value
        }

    var targetPower = 0.0

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            // asymmetrically slew rate power
            val slewRate = if (sign(_power) == sign(targetPower)) accelerationRate else decelerationRate
//            _power += (targetPower - _power).clamp(-slewRate, slewRate)
            _power = targetPower

            motor.power = _power
        }
    }


    companion object {
        @JvmField
        var motorName = "m1e"

        @JvmField
        var motorReversed = true

        @JvmField
        var accelerationRate = .1

        @JvmField
        var decelerationRate = .03
    }
}