package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.Pose2d
import org.firstinspires.ftc.teamcode.common.hardware.AnalogEncoder
import org.firstinspires.ftc.teamcode.common.hardware.Encoder
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.sign

/**
 * Everything should be stored in normalized degrees.
 * Unless it's something that needs to be unnormalized for wraparound reasons.
 */
@Configurable
class Turret(opMode: OpMode, var isTeleop: Boolean = false, val getPose: (() -> Pose2d)? = null) : Subsystem("Turret") {
    val motor = ManualMotor(opMode.hardwareMap, motorName, reverse = true)
    val motorEncoder = Encoder(opMode.hardwareMap, motorEncoderName, reverse = true)
    val analog = AnalogEncoder(opMode.hardwareMap, analogEncoderName, reverse = true).apply {
        update()
    }

    val pid = PIDController(kp, ki, kd)

    private val startingOffset = normalizeDegrees(analog.rawPosition * 360.0 + 90)
    val driveHeading
        get() = getPose?.invoke()?.heading ?: 0.0
    val relativeAngle
        get() = ticksToDegrees(motorEncoder.position) + startingOffset
    val globalAngle // hk this is annoying, we can't just use this for pid, as it screws up wraparound
        get() = normalizeDegrees(driveHeading + relativeAngle)

    var targetingMode: TargetingMode = TargetingMode.RELATIVE
    var targetAngle: Double = 0.0
        set(value) {
            field = normalizeDegrees(value)
        }

    private fun ticksToDegrees(ticks: Double): Double {
        return ticks * GEAR_RATIO / PPR * 360.0
    }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                // this is in relative space for wraparound reasons
                val targetAngle: Double = when (targetingMode) {
                    TargetingMode.RELATIVE -> targetAngle
                    TargetingMode.GLOBAL -> normalizeDegrees(targetAngle - driveHeading)
                }

                val pidOutput = pid.calculate(relativeAngle, targetAngle)
                val ff = ks * sign(targetAngle - relativeAngle)
                val power = (pidOutput + ff).clamp(-maxPower, maxPower)
                val boostedPower = if (power > 0.0) power * boostMulti else power
                motor.power = boostedPower

                tel.addData("Turret | relativeAngle", relativeAngle)
                tel.addData("Turret | globalAngle", globalAngle)
                tel.addData("Turret | targetAngle", this@Turret.targetAngle)
                tel.addData("Turret | relativeTargetAngle", targetAngle)
                tel.addData("Turret | power", boostedPower)
                sync()
            }
        }
    }

    enum class TargetingMode {
        RELATIVE,
        GLOBAL;
    }

    companion object {
        @JvmField
        var motorName = "m0e"

        @JvmField
        var motorEncoderName = "m0e"

        @JvmField
        var analogEncoderName = "a2"

        const val PPR = ((1 + (46.0 / 17.0)) * 28.0)

        const val GEAR_RATIO = 24.0 / 110.0

        @JvmField
        var kp = -0.037

        @JvmField
        var ki = 0.0

        @JvmField
        var kd = 0.0

        @JvmField
        var ks = 0.0

        @JvmField
        var maxPower = 0.6

        @JvmStatic
        var boostMulti = 1.5
    }
}
