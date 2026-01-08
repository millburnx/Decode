package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.AnalogEncoder
import org.firstinspires.ftc.teamcode.common.hardware.Encoder
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.sign

@Configurable
class Turret(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Turret") {
    val motor = ManualMotor(opMode.hardwareMap, motorName)
    val motorEncoder = Encoder(opMode.hardwareMap, motorEncoderName)
    val analog = AnalogEncoder(opMode.hardwareMap, analogEncoderName).apply {
        update()
    }

    val encoderOffset = analog.rawPosition / GEAR_RATIO * PPR - motorEncoder.position
    val correctedPosition
        get() = motorEncoder.position + encoderOffset

    val pid = PIDController(kp, ki, kd)

    var targetAngle = 0.0
    val targetPosition
        get() = targetAngle / 360.0 / GEAR_RATIO * PPR

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                pid.setPID(kp, ki, kd)
                val pidOutput = pid.calculate(correctedPosition, targetPosition)
                val ff = ks * sign(pidOutput)
                val power = (pidOutput + ff).clamp(-maxPower, maxPower)
                motor.power = power

                analog.update()

                tel.addData("Turret | Power", power)
                tel.addData("Turret | Target", targetPosition)
                tel.addData("Turret | Current", correctedPosition)
                tel.addData("Turret | Current Normalized", correctedPosition * GEAR_RATIO / PPR)
                tel.addData("Turret | Raw", motor.position)
                tel.addData("Turret | Raw analog", analog.rawPosition)
                tel.addData("Turret | Offset", encoderOffset)
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var motorName = "m0e"

        @JvmField
        var motorEncoderName = "m0e"

        @JvmField
        var analogEncoderName = "a2"

        @JvmField
        var kp = 0.05

        @JvmField
        var ki = 0.0

        @JvmField
        var kd = 0.0

        @JvmField
        var ks = 0.0

        @JvmField
        var maxPower = 0.6

        const val PPR = ((1 + (46.0 / 17.0)) * 28.0)

        const val GEAR_RATIO = 24.0/110.0

        @JvmStatic
        var negMulti = 1.5
    }
}
