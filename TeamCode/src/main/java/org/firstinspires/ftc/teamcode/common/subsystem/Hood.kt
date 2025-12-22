package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualServo
import org.firstinspires.ftc.teamcode.opmode.OpMode


class Hood(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Hood") {
    val servo = ManualServo(opMode.hardwareMap, servoName, reverse = servoReversed)

    var position = min

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    var change = 0.0
                    if (gp2.current.rightBumper && !gp2.prev.rightBumper) change = bumperStep
                    if (gp2.current.leftBumper && !gp2.prev.leftBumper) change = -bumperStep
                    position = (position + change).coerceIn(min, max)
                    tel.addData("hood | pos", position)
                }
                servo.position = position
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var servoName = "s0"

        @JvmField
        var servoReversed = false

        @JvmField
        var bumperStep = 0.1

        @JvmField
        var min = 0.0

        @JvmField
        var max = 0.5
    }
}