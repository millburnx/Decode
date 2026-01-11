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
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Drawing
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

    private val startingOffset = -normalizeDegrees(analog.rawPosition * 360.0 - analogOffset)
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
        println("meow $startingOffset ${analog.rawPosition}")

        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                pid.setPID(kp, ki, kd)

                // this is in relative space for wraparound reasons
                val immutableTargetingMode = targetingMode
                if (immutableTargetingMode != TargetingMode.OFF) {
                    val targetAngle: Double = convertToWraparound(when (immutableTargetingMode) {
                        TargetingMode.RELATIVE -> targetAngle
                        TargetingMode.GLOBAL -> normalizeDegrees(targetAngle - driveHeading)
                        else -> throw Error("Unexpected targeting mode: $immutableTargetingMode")
                    })
                    val pidOutput = pid.calculate(relativeAngle, targetAngle)
                    val ff = ks * sign(targetAngle - relativeAngle)
                    val power = (pidOutput + ff).clamp(-maxPower, maxPower)
                    val boostedPower = if (power > 0.0) power * boostMulti else power
                    motor.power = boostedPower
                    tel.addData("Turret | power", boostedPower)
                } else {
                    motor.power = 0.0
                }

                val pose = getPose?.invoke()
                if (pose != null) {
                    Drawing.drawRobot(
                        pose.toPedro(), turretAngle = globalAngle
                    )
                }

                tel.addData("Turret | relativeAngle", relativeAngle)
                tel.addData("Turret | globalAngle", globalAngle)
                tel.addData("Turret | targetAngle", this@Turret.targetAngle)
                tel.addData("Turret | relativeTargetAngle", targetAngle)
                sync()
            }
        }
    }

    enum class TargetingMode {
        OFF, RELATIVE, GLOBAL;
    }

    // Make angle wraparound friendly, -180, 180 to -270, 90
    private fun convertToWraparound(angle: Double): Double {
        if (angle <= 90 || angle < 0.0) return angle
        return angle - 360
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

        @JvmStatic
        var analogOffset = 70
    }
}