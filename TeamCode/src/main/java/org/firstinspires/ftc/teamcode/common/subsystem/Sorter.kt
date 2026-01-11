package org.firstinspires.ftc.teamcode.common.subsystem

import androidx.core.graphics.green
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.qualcomm.hardware.rev.RevColorSensorV3
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class Sorter(opMode: OpMode) : Subsystem("Sorter") {
    val c0 = opMode.hardwareMap.colorSensor["c0"]
    val c1 = opMode.hardwareMap["c1"] as RevColorSensorV3
    val c2 = opMode.hardwareMap.colorSensor["c2"]

    val c3 = opMode.hardwareMap["c0e"] as RevColorSensorV3
    val c4 = opMode.hardwareMap.colorSensor["c1e"]
    val c5 = opMode.hardwareMap["c2e"] as RevColorSensorV3

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                val color0 = c0.argb()
                val color2 = c2.argb()
                val color4 = c4.argb()

                val color1 = c1.argb()
                val color3 = c3.argb()
                val color5 = c5.argb()

                val stateRight = if (color0.green >= c0g) {
                    State.Green
                } else if (color0.green >= c0p) {
                    State.Purple
                } else if (color1.green >= c1g) {
                    State.Green
                } else if (color1.green >= c1p) {
                    State.Purple
                } else {
                    State.Empty
                }

                val stateFront = if (color4.green >= c4g) {
                    State.Green
                } else if (color4.green >= c4p) {
                    State.Purple
                } else if (color3.green >= c3g) {
                    State.Green
                } else if (color3.green >= c3p) {
                    State.Purple
                } else {
                    State.Empty
                }

                val stateBack = if (color2.green >= c2g) {
                    State.Green
                } else if (color2.green >= c2p) {
                    State.Purple
                } else if (color5.green >= c5g) {
                    State.Green
                } else if (color5.green >= c5p) {
                    State.Purple
                } else {
                    State.Empty
                }

                tel.addData("right", stateRight)
                tel.addData("front", stateFront)
                tel.addData("back", stateBack)
                sync()
            }
        }
    }

    enum class State {
        Empty,
        Green,
        Purple
    }

    companion object {
        @JvmField
        var c0p = 67

        @JvmField
        var c0g = 87

        @JvmField
        var c1p = 34

        @JvmField
        var c1g = 35

        @JvmField
        var c2p = 34

        @JvmField
        var c2g = 39

        @JvmField
        var c3p = 23

        @JvmField
        var c3g = 24

        @JvmField
        var c4p = 34

        @JvmField
        var c4g = 40

        @JvmField
        var c5p = 13

        @JvmField
        var c5g = 14
    }
}