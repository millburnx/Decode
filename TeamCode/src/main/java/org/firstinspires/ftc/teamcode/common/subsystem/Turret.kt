package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.util.toRadians
import org.firstinspires.ftc.teamcode.common.hardware.AnalogEncoder
import org.firstinspires.ftc.teamcode.common.hardware.Encoder
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleopManager.Companion.rotationVelocityThreshold
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.common.util.PIDFCoefficients
import org.firstinspires.ftc.teamcode.common.util.TimeAverage
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.opmode.test.pedro.StandaloneRotation
import org.firstinspires.ftc.teamcode.pedro.DriftKalmanFilter
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign
import com.pedropathing.control.PIDFCoefficients as PedroPIDFCoeff

/**
 * Everything here is within the internal range
 * Input is to be transformed into the internal range
 */
@Configurable
class Turret(
    opMode: OpMode,
    val heading: () -> Double,
    val velocity: () -> Double,
    val voltage: () -> Double,
) : Subsystem("Turret") {
    val analog = AnalogEncoder(opMode.hardwareMap, analogName, analogReversed)
    val quadature = Encoder(opMode.hardwareMap, quadatureName, quadratureReversed)
    val motor = ManualMotor(opMode.hardwareMap, motorName, motorReversed)

    val startingAnalog =
        run {
            analog.update()
            (analog.rawPosition - 0.5) * 360.0
        }
        get() = field + startingOffset

    var driftKF =
        DriftKalmanFilter {
            kfConfig
        }

    private val _angle
        get() = quadature.position * TICKS_TO_DEGREES + startingAnalog + driftKF.drift

    val quadatureVelocity
        get() = quadature.velocity

    val averagedQuadatureVelocity = TimeAverage { velocityAverageDuration }

    val angle
        get() = normalizeDegrees(_angle + 180.0)

    var target = 0.0
        set(value) {
            field = normalizeDegrees(value)
        }

    var targetingMode = TargetingMode.RELATIVE

    val isOff
        get() = targetingMode == TargetingMode.OFF
    val isRelative
        get() = targetingMode == TargetingMode.RELATIVE
    val isGlobal
        get() = targetingMode == TargetingMode.GLOBAL

    fun setOff() {
        targetingMode = TargetingMode.OFF
    }

    fun setRelative() {
        targetingMode = TargetingMode.RELATIVE
    }

    fun setGlobal() {
        targetingMode = TargetingMode.GLOBAL
    }

    private val relativeTarget
        get() =
            when (targetingMode) {
                TargetingMode.OFF -> {
                    Double.NEGATIVE_INFINITY
                }

                TargetingMode.RELATIVE -> {
                    target
                }

                TargetingMode.GLOBAL -> {
                    val heading = heading()
                    normalizeDegrees(target - heading)
                }
            }

    private val _target
        get() = normalizeDegrees(relativeTarget - 180.0).clamp(min, max)

    val atTarget: Boolean
        get() {
            if (relativeTarget == Double.NEGATIVE_INFINITY) return false
            return abs(normalizeDegrees(_angle - _target)) < threshold
        }

    val inDeadzone
        get() = normalizeDegrees(relativeTarget - 180.0) !in min..max

    val isSteady
        get() = averagedQuadatureVelocity.average < steadyThreshold

    val isReady: Boolean
        get() {
            val isSteady = isSteady && velocity() < rotationVelocityThreshold
            return atTarget && !inDeadzone && isSteady
        }

    val controller =
        StandaloneRotation(
            useWraparound = false,
            { PedroPIDFCoeff(coeff.kP, coeff.kI, coeff.kD, coeff.kS) },
            { PedroPIDFCoeff(coeffSecondary.kP, coeffSecondary.kI, coeffSecondary.kD, coeffSecondary.kS) },
            { useSecondary },
        )

    override val run: suspend Command.()
    -> Unit =
        {
            OpModeLoop(opMode) {
                with(opMode) {
                    averagedQuadatureVelocity.update(abs(quadatureVelocity))
                    if (targetingMode == TargetingMode.OFF) {
                        motor.power = 0.0
                    } else {
                        val driveCompFF = if (targetingMode == TargetingMode.GLOBAL) kR * -velocity() else 0.0
                        val pidf = controller.calc(_angle.toRadians(), _target.toRadians()) + driveCompFF
                        val ks = sign(pidf) * coeff.kS

                        val rawPower = pidf
                        val power =
                            if (abs(rawPower) < minPower) rawPower * abs(rawPower).pow(2) / minPower.pow(2) else rawPower

                        val voltage = voltage()
                        val voltageComp = if (voltage != 0.0) (12.0 / voltage) else 1.0

                        motor.power = power * voltageComp

                        if (useTelemetry) {
                            tel.addData("turret | power", power)
                            tel.addData("turret | kv", driveCompFF)
                        }
                    }
                    if (useTelemetry) {
                        tel.addData("turret | dv", velocity())
                        tel.addData("turret | qv", quadatureVelocity)
                        tel.addData("turret | aqv", averagedQuadatureVelocity.average)

                        tel.addData("turret | ea", angle)
                        tel.addData("turret | ra", heading())
                        tel.addData("turret | ta", normalizeDegrees(angle + heading()))
                        tel.addData("turret | it", _target)
                        tel.addData("turret | rt", relativeTarget)
                        tel.addData("turret | et", target)
                        tel.addData("turret | drift", driftKF.drift)
                        tel.addData("turret | ia", _angle)
                        tel.addData("turret | ia (w/o)", _angle - driftKF.drift)
                        tel.addData("turret | at target", atTarget)
                        tel.addData("turret | in deadzone", inDeadzone)
                    }

                    analog.update()
                    driftKF.update(_angle - driftKF.drift, approxAnalog())
                }
            }
        }

    fun approxAnalog(): Double {
        val analogAngle = normalizeDegrees((analog.rawPosition - 0.5) * 360.0 + startingOffset)
        // since analog doesn't count analog, lowkey just see if base, base - 360.0, or base + 360.0 is closest to the current angle
        val candidates = listOf(analogAngle, analogAngle - 360.0, analogAngle + 360.0)
        val closest = candidates.minBy { abs(it - _angle) }
        return closest
    }

    enum class TargetingMode {
        OFF,
        RELATIVE,
        GLOBAL,
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
        var min = -155.0

        @JvmField
        var max = 160.0

        // gear ratio / ppr * 360.0
        const val TICKS_TO_DEGREES = 0.2 / ((1.0 + (46.0 / 11.0)) * 28.0) * 360

        @JvmField
        var coeff = PIDFCoefficients(0.75, 0.0, 0.075, 0.05)

        @JvmField
        var coeffSecondary = PIDFCoefficients(3.0, 0.0, 0.001, 0.05)

        @JvmField
        var useSecondary = true

        @JvmField
        var kR = 0.0

        @JvmField
        var minPower = 0.0

        @JvmField
        var startingOffset = -4.0

        @JvmField
        var threshold = 10.0

        @JvmField
        var steadyThreshold = 50.0

        @JvmField
        var velocityAverageDuration = 250.0

        @JvmField
        var kfConfig =
            DriftKalmanFilter.Config(
                startingUncertainty = .01,
                processNoise = 0.01, // q
                measurementNoise = 1.0, // r
                maxGain = 0.1,
                maxDist = 10.0,
                minUncertainty = 1e-3,
            )

        @JvmField
        var useTelemetry = false
    }
}
