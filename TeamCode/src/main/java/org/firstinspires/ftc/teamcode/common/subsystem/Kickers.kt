package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualServo
import org.firstinspires.ftc.teamcode.opmode.OpMode


class Kickers(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Kickers") {
    val servo1 = ManualServo(opMode.hardwareMap, servo1Name, reverse = servo1Reversed)
    val servo2 = ManualServo(opMode.hardwareMap, servo2Name, reverse = servo2Reversed)
    val servo3 = ManualServo(opMode.hardwareMap, servo3Name, reverse = servo3Reversed)

    var isUp1 = false
    var isUp2 = false
    var isUp3 = false

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    if (gp1.current.dPad.left && !gp1.prev.dPad.left) isUp1 = !isUp1
                    if (gp1.current.dPad.up && !gp1.prev.dPad.up) isUp2 = !isUp2
                    if (gp1.current.dPad.right && !gp1.prev.dPad.right) isUp3 = !isUp3
                }
                servo1.position = if (isUp1) up1Position else idle1Position
                servo2.position = if (isUp2) up2Position else idle2Position
                servo3.position = if (isUp3) up3Position else idle3Position
                tel.addData("kickers | s1", isUp1)
                tel.addData("kickers | s2", isUp2)
                tel.addData("kickers | s3", isUp3)
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var servo1Name = "s1"

        @JvmField
        var servo2Name = "s2"

        @JvmField
        var servo3Name = "s3"

        @JvmField
        var servo1Reversed = false

        @JvmField
        var servo2Reversed = false

        @JvmField
        var servo3Reversed = false

        @JvmField
        var idle1Position = 0.8

        @JvmField
        var idle2Position = 0.8

        @JvmField
        var idle3Position = 0.8

        @JvmField
        var up1Position = 0.3

        @JvmField
        var up2Position = 0.3

        @JvmField
        var up3Position = 0.3
    }
}