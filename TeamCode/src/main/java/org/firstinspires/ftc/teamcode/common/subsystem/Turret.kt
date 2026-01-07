package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDController
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.toDegrees
import org.firstinspires.ftc.teamcode.common.hardware.AnalogEncoder
import org.firstinspires.ftc.teamcode.common.hardware.Encoder
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.atan2
import kotlin.math.sign

@Configurable
class Turret(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Turret") {
    val motor = ManualMotor(opMode.hardwareMap, motorName)
    val motorEncoder = Encoder(opMode.hardwareMap, motorEncoderName)
    val analog = AnalogEncoder(opMode.hardwareMap, analogEncoderName) { motorEncoder.velocity }

    val pid = PIDController(kp, ki, kd)

    var targetPosition = 0.0

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                analog.update()
                if (isTeleop && !override) {
                    val targetAngle = atan2(-gp2.current.rightJoyStick.y, gp2.current.rightJoyStick.x)
                    targetPosition = targetAngle.toDegrees() / 360.0 + 0.5
                } else {
                    targetPosition = overridePosition
                }
                pid.setPID(kp, ki, kd)
                val pidPower= pid.calculate(analog.position, targetPosition)
                val power = pidPower + (ks * sign(pidPower))
                val clampedPower =  power.coerceIn(-maxPower, maxPower)
                val finalPower = if (clampedPower < 0.0) clampedPower * negMulti else clampedPower
                motor.power = finalPower
                tel.addData("Turret | power", power)
                tel.addData("Turret | clamped power", power.coerceIn(-maxPower, maxPower))
                tel.addData("Turret | final power", finalPower)
                tel.addData("Turret | Target", targetPosition)
                tel.addData("Turret | Current", analog.position)
                tel.addData("Turret | Current Motor", motor.position)
                tel.addData("Turret | Velocity", motorEncoder.velocity)
                tel.addData("Turret | RPM", motorEncoder.velocity * 60.0 / PPR * GEAR_RATIO)
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
        var kp = 5.0

        @JvmField
        var ki = 0.0

        @JvmField
        var kd = 0.0

        @JvmField
        var ks = 0.0

        @JvmField
        var maxPower = 0.6

        const val PPR = ((1 + (46.0 / 17.0)) * 28.0)

        const val GEAR_RATIO = 24/110

        @JvmStatic
        var override = true
        @JvmStatic
        var overridePosition = 0.5

        @JvmStatic
        var negMulti = 1.5
    }
}
