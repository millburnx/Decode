package org.firstinspires.ftc.teamcode.common.subsystem.sorter

import android.graphics.Color
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualServo
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class SorterPod(val opMode: OpMode, val getConfig: () -> Config) : Subsystem("Sorter Pod") {
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

//                updateState()
                if (config.useTelemetry) {
                    tel.addData("state", state)
                }
            }
        }
    }

    suspend fun Command.kick(
        durations: Pair<Long, Long>
    ) {
        val (upDuration, downDuration) = durations
        isUp = true
        SleepFor { if (config.isAxon) Config.axonDurationUp else upDuration }
        isUp = false
        SleepFor { if (config.isAxon) Config.axonDurationDown else downDuration }
    }

    fun updateState() {
        with(opMode) {
            // v3
            v3.gain = config.v3Gain.toFloat()

            val v3Color = v3.normalizedColors.toHSV()

            if (config.useTelemetry) {
                tel.addData("v3 | h", v3Color.hue)
                tel.addData("v3 | s", v3Color.saturation)
                tel.addData("v3 | v", v3Color.brightness)
            }

            if (config.greenRangeV3.contains(v3Color)) {
                state = State.GREEN
                return
            }
            if (config.purpleRangeV3.contains(v3Color)) {
                state = State.PURPLE
                return
            }

            state = State.EMPTY

            // v2 fallback
//            val v2Color = v2.argb().hsv()
//
//            state = if (config.greenRangeV2.contains(v2Color)) {
//                State.GREEN
//            } else if (config.purpleRangeV2.contains(v2Color)) {
//                State.PURPLE
//            } else {
//                State.EMPTY
//            }
//
//            if (config.useTelemetry) {
//                tel.addData("v2 | h", v2Color.hue)
//                tel.addData("v2 | s", v2Color.saturation)
//                tel.addData("v2 | v", v2Color.brightness)
//            }
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

        @JvmField // done
        var frontConfig = Config(
            "s0", false, .4, .8, "c0e", "c2e", 10.0, ColorRange(
                150.0..155.0, 27.0..30.5, 19.0..25.0
            ), ColorRange(
                165.0..172.0, 40.0..45.0, 66.0..70.0
            ), ColorRange(
                155.5..160.0, 34.5..37.0, 19.0..25.0
            ), ColorRange(
                158.0..165.0, 47.0..50.0, 68.0..74.0
            ), false
        )

        @JvmField
        var backConfig = Config(
            "s3", false, .35, .67, "c1e", "c0", 10.0, ColorRange(
                154.0..159.0, 30.0..33.0, 15.0..19.0
            ), ColorRange(
                145.5..152.0, 39.0..42.0, 40.0..42.0
            ), ColorRange(
                153.0..157.0, 35.0..38.0, 15.0..19.0
            ), ColorRange(
                145.0..152.5, 44.0..47.0, 42.0..45.0
            ), false, true
        )

        @JvmField
        var sideConfig = Config(
            "s4", true, .30, .70, "c1", "c2", 10.0, ColorRange(
                160.0..164.0,
                10.0..20.0,
                26.0..29.0,
            ), ColorRange(
                162.0..170.0,
                40.0..46.0,
                74.5..78.0
            ), ColorRange(
                143.0..159.0,
                10.0..20.0,
                32.0..36.0,
            ), ColorRange(
                155.0..162.0,
                46.0..52.0,
                76.0..80.0,
            ), false
        )
    }

    enum class Slot {
        FRONT, BACK, SIDE
    }

    enum class State {
        EMPTY, PURPLE, GREEN,
    }

    @Configurable

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

        var isAxon: Boolean = false


    ) {
        companion object {
            @JvmField
            var axonDurationUp = 500L

            @JvmField
            var axonDurationDown = 100L
        }
    }
}

class ColorRange(
    var minHue: Double,
    var maxHue: Double,
    var minSaturation: Double,
    var maxSaturation: Double,
    var minValue: Double,
    var maxValue: Double
) {
    fun contains(color: HSV): Boolean {
        return color.hue in minHue..maxHue && color.saturation in minSaturation..maxSaturation && color.brightness in minValue..maxValue
    }

    constructor(
        hueRange: ClosedFloatingPointRange<Double>,
        saturationRange: ClosedFloatingPointRange<Double>,
        valueRange: ClosedFloatingPointRange<Double>
    ) : this(
        hueRange.start,
        hueRange.endInclusive,
        saturationRange.start,
        saturationRange.endInclusive,
        valueRange.start,
        valueRange.endInclusive
    )
}

data class HSV(
    val hue: Double,
    val saturation: Double,
    val brightness: Double,
)

fun NormalizedRGBA.toHSV(): HSV {
    val hsv = FloatArray(3)
    Color.colorToHSV(this.toColor(), hsv)
    return HSV(hsv[0] * 1.0, hsv[1] * 100.0, hsv[2] * 100.0)
}

fun Int.hsv(): HSV {
    val hsv = FloatArray(3)
    Color.colorToHSV(this, hsv)
    return HSV(hsv[0] * 1.0, hsv[1] * 100.0, hsv[2] * 100.0)
}