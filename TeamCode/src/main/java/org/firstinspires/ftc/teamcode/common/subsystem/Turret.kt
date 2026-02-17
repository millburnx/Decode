package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.hardware.AnalogEncoder
import org.firstinspires.ftc.teamcode.common.hardware.Encoder
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.common.util.PIDFCoefficients
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.abs
import kotlin.math.sign

/**
 * Everything here is within the internal range
 * Input is to be transformed into the internal range
 */
@Configurable
class Turret(opMode: OpMode) : Subsystem("Turret") {
    val analog = AnalogEncoder(opMode.hardwareMap, analogName, analogReversed)
    val quadature = Encoder(opMode.hardwareMap, quadatureName, quadratureReversed)
    val motor = ManualMotor(opMode.hardwareMap, motorName, motorReversed)

    val startingAnalog = run {
        analog.update()
        (analog.rawPosition - 0.5) * 360.0
    }

    private val _angle
        get() = quadature.position * TICKS_TO_DEGREES + startingAnalog

    val angle
        get() = normalizeDegrees(_angle + 180.0)

    var target = 0.0
        set(value) {
            field = normalizeDegrees(value)
        }

    private val _target
        get() = normalizeDegrees(target - 180.0).clamp(min, max)

    val pidf = PIDController(coeff.kP, coeff.kI, coeff.kD)

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            with(opMode) {
                analog.update()
                pidf.setPID(coeff.kP, coeff.kI, coeff.kD)

                val ff = sign( _target - _angle) * coeff.kS

                val rawPower = pidf.calculate(_angle, _target) + ff
                val power = if (abs(rawPower) < minPower) 0.0 else rawPower
                motor.power = power

                tel.addData("turret | ia", _angle)
                tel.addData("turret | ea", angle)
                tel.addData("turret | it", _target)
                tel.addData("turret | et", target)
                tel.addData("turret | power", power)
            }
        }
    }

    companion object {
        @JvmField
        var analogName: String = "a0"

        @JvmField
        var quadatureName: String = "m0e"

        @JvmField
        var motorName: String = "m0e"

        @JvmField
        var analogReversed: Boolean = true

        @JvmField
        var quadratureReversed: Boolean = false

        @JvmField
        var motorReversed: Boolean = true

        @JvmField
        var min = -170.0

        @JvmField
        var max = 160.0

        // gear ratio / ppr * 360.0
        const val TICKS_TO_DEGREES = 0.2 / ((1.0 + (46.0 / 11.0)) * 28.0) * 360

        @JvmField
        var coeff = PIDFCoefficients(0.05, 0.0, 0.0, 0.05)

        @JvmField
        var minPower = 0.2
    }
}