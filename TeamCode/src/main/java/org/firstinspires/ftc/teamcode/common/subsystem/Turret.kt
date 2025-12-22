package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDFController
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.toDegrees
import org.firstinspires.ftc.teamcode.common.hardware.AnalogEncoder
import org.firstinspires.ftc.teamcode.common.hardware.Encoder
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.atan2

class Turret(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Turret") {
    val motor = ManualMotor(opMode.hardwareMap, motorName)
    val motorEncoder = Encoder(opMode.hardwareMap, motorEncoderName)
    val analog = AnalogEncoder(opMode.hardwareMap, analogEncoderName) { motorEncoder.velocity }

    val pidf = PIDFController(kp, ki, kd, ks)

    var targetPosition = 0.0

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    val targetAngle = atan2(-gp2.current.rightJoyStick.y, gp2.current.rightJoyStick.x)
                    targetPosition = targetAngle.toDegrees() / 360.0
                }
                pidf.setPIDF(kp, ki, kd, ks)
                motor.power = pidf.calculate(analog.position, targetPosition)
                tel.addData("Turret | Target", targetPosition)
                tel.addData("Turret | Current", analog.position)
                tel.addData("Turret | Velocity", motorEncoder.velocity)
                tel.addData("Turret | RPM", motorEncoder.velocity * 60.0 / PPR * GEAR_RATIO)
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var motorName = "m3e"

        @JvmField
        var motorEncoderName = "m0"

        @JvmField
        var analogEncoderName = "a0"

        @JvmField
        var kp = 0.0

        @JvmField
        var ki = 0.0

        @JvmField
        var kd = 0.0

        @JvmField
        var ks = 0.0

        const val PPR = ((1 + (46.0 / 17.0)) * 28.0)

        const val GEAR_RATIO = 24/110
    }
}
