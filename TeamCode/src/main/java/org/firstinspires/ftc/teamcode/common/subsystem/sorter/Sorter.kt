package org.firstinspires.ftc.teamcode.common.subsystem.sorter

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualServo
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class Sorter(val opMode: OpMode) : Subsystem("FlyWheel") {

    val back = ManualServo(opMode.hardwareMap, backServoName, backServoReversed)
    val front = ManualServo(opMode.hardwareMap, frontServoName, frontServoReversed)
    val side = ManualServo(opMode.hardwareMap, sideServoName, sideServoReversed)

    var isBackUp = false
    var isFrontUp = false
    var isSideUp = false

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            with(opMode) {
                if (!gp1.prev.dPad.left && gp1.current.dPad.left) {
                    isBackUp = !isBackUp
                }
                if (!gp1.prev.dPad.up && gp1.current.dPad.up) {
                    isFrontUp = !isFrontUp
                }
                if (!gp1.prev.dPad.right && gp1.current.dPad.right) {
                    isSideUp = !isSideUp
                }

                back.position = if (isBackUp) backUp else backDown
                front.position = if (isFrontUp) frontUp else frontDown
                side.position = if (isSideUp) sideUp else sideDown

                tel.addData("isBackUp", isBackUp)
                tel.addData("isFrontUp", isFrontUp)
                tel.addData("isSideUp", isSideUp)
            }
        }
    }

    companion object {
        @JvmField
        var backServoName = "s3"

        @JvmField
        var frontServoName = "s5"

        @JvmField
        var sideServoName = "s4"

        @JvmField
        var backServoReversed = true

        @JvmField
        var frontServoReversed = false

        @JvmField
        var sideServoReversed = true

        @JvmField
        var backDown = .41

        @JvmField
        var frontDown = .4

        @JvmField
        var sideDown = .4

        @JvmField
        var backUp = .8

        @JvmField
        var frontUp = .8

        @JvmField
        var sideUp = .8
    }
}