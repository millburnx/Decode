package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.hardware.lerp
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class Hood(opMode: OpMode) : Subsystem("Hood") {
//    val servo = ManualServo(opMode.hardwareMap, servoName, servoReversed)

    var target = 0.0
        set(value) {
            field = value.clamp(0.0, 1.0)
        }

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            val targetPos = lerp(min, max, target)
//            servo.position = targetPos
        }
    }

    companion object {
        @JvmField
        var servoName = "s2"

        @JvmField
        var servoReversed = false

        @JvmField
        var min: Double = 0.0

        @JvmField
        var max: Double = 1.0
    }
}