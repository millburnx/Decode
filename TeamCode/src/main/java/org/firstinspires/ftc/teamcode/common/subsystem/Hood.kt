package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualServo
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class Hood(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Hood") {
    val servo = ManualServo(opMode.hardwareMap, servoName, reverse = servoReversed)

    /**
     * Hood position, range: 0-1
     */
    var position = 0.0
        set(value) {
            field = value.clamp(0.0, 1.0)
        }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    var change = 0.0
                    if (gp2.current.rightBumper && !gp2.prev.rightBumper) change = bumperStep
                    if (gp2.current.leftBumper && !gp2.prev.leftBumper) change = -bumperStep
                    position += change
                    tel.addData("hood | pos", position)
                }
                servo.position = min + position * (max - min)
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var servoName = "s0e"

        @JvmField
        var servoReversed = false

        @JvmField
        var bumperStep = 0.1

        @JvmField
        var min = 0.3

        @JvmField
        var max = 0.9
    }
}