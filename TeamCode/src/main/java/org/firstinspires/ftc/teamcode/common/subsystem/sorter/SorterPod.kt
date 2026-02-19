package org.firstinspires.ftc.teamcode.common.subsystem.sorter

import android.graphics.Color
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualServo
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class SorterPod(val opMode: OpMode, val getConfig: () -> Config) : Subsystem("Sorter") {
    val config // for hot reloading
        get() = getConfig()

    val servo = ManualServo(opMode.hardwareMap, config.servoName, config.servoReversed)
    val v2 = opMode.hardwareMap.colorSensor[config.v2Name]
    val v3 = opMode.hardwareMap[config.v3Name] as RevColorSensorV3

    var isUp = false
    var state = State.EMPTY

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            with(opMode) {
                servo.position = if (isUp) {
                    config.upPosition
                } else {
                    config.downPosition
                }

                updateState()
            }
        }
    }

    fun updateState() {
        with(opMode) {
            // v3
            v3.gain = config.v3Gain.toFloat()

            val v3Color = v3.normalizedColors.toHSV()

            tel.addData("hsv v3", v3Color)

            if (config.greenRangeV3.contains(v3Color)) {
                state = State.GREEN
                return
            }
            if (config.purpleRangeV3.contains(v3Color)) {
                state = State.PURPLE
                return
            }

            // v2 fallback
            val v2Color = v2.argb().hsv()

            state = if (config.greenRangeV2.contains(v2Color)) {
                State.GREEN
            } else if (config.purpleRangeV2.contains(v2Color)) {
                State.PURPLE
            } else {
                State.EMPTY
            }

            tel.addData("hsv v2", v2Color)
            tel.addData("state", state)
        }
    }

    companion object {
        fun fromSlot(opMode: OpMode, slot: Slot): SorterPod {
            val config = when (slot) {
                Slot.FRONT -> frontConfig
                Slot.BACK -> backConfig
                Slot.SIDE -> sideConfig
            }
            return SorterPod(opMode) { config }
        }

        @JvmField
        var frontConfig = Config(
            "s5",
            false,
            .4,
            .8,
            "c1e",
            "c2e",
            1.0,
            ColorRange(240.0..300.0, 35.0..100.0),
            ColorRange(240.0..300.0, 35.0..100.0),
            ColorRange(70.0..160.0, 35.0..100.0),
            ColorRange(70.0..160.0, 35.0..100.0),
            true
        )

        @JvmField
        var backConfig = Config(
            "s3",
            true,
            .41,
            .8,
            "c0e",
            "c2",
            1.0,
            ColorRange(240.0..300.0, 35.0..100.0),
            ColorRange(240.0..300.0, 35.0..100.0),
            ColorRange(70.0..160.0, 35.0..100.0),
            ColorRange(70.0..160.0, 35.0..100.0),
            false
        )

        @JvmField
        var sideConfig = Config(
            "s4",
            true,
            .4,
            .8,
            "c0",
            "c1",
            1.0,
            ColorRange(240.0..300.0, 35.0..100.0),
            ColorRange(240.0..300.0, 35.0..100.0),
            ColorRange(70.0..160.0, 35.0..100.0),
            ColorRange(70.0..160.0, 35.0..100.0),
            false
        )
    }

    enum class Slot {
        FRONT, BACK, SIDE
    }

    enum class State {
        EMPTY, PURPLE, GREEN,
    }

    class Config(
        var servoName: String,
        var servoReversed: Boolean,

        var downPosition: Double,
        var upPosition: Double,

        var v2Name: String,
        var v3Name: String,

        var v3Gain: Double,

        var purpleRangeV2: ColorRange,
        var purpleRangeV3: ColorRange,

        var greenRangeV2: ColorRange,
        var greenRangeV3: ColorRange,

        var useTelemetry: Boolean,
    )
}

class ColorRange(
    var minHue: Double,
    var maxHue: Double,
    var minSaturation: Double,
    var maxSaturation: Double,
) {
    fun contains(color: HSV): Boolean {
        return color.hue in minHue..maxHue && color.saturation in minSaturation..maxSaturation
    }

    constructor(
        hueRange: ClosedFloatingPointRange<Double>,
        saturationRange: ClosedFloatingPointRange<Double>,
    ) : this(hueRange.start, hueRange.endInclusive, saturationRange.start, saturationRange.endInclusive)
}

data class HSV(
    val hue: Double,
    val saturation: Double,
    val brightness: Double,
)

fun NormalizedRGBA.toHSV(): HSV {
    val hsv = FloatArray(3)
    Color.colorToHSV(this.toColor(), hsv)
    return HSV(hsv[0] * 360.0, hsv[1] * 100.0, hsv[2] * 100.0)
}

fun Int.hsv(): HSV {
    val hsv = FloatArray(3)
    Color.colorToHSV(this, hsv)
    return HSV(hsv[0] * 360.0, hsv[1] * 100.0, hsv[2] * 100.0)
}