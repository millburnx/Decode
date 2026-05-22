package org.firstinspires.ftc.teamcode.common.subsystem.sorter

import android.graphics.Color
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualServo
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class SorterPod(val opMode: OpMode, val getConfig: () -> Config) : Subsystem("Sorter Pod") {
    val config // for hot reloading
        get() = getConfig()

    val servo = ManualServo(opMode.hardwareMap, config.servoName, config.servoReversed)
    val v2 = opMode.hardwareMap[config.v2Name] as ColorRangeSensor
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

                updateState() // TODO: Implement polling & caching once working
                if (config.useTelemetry) tel.addData("pod | state", state)
            }
        }
    }

    suspend fun Command.kick(
        durations: Pair<Long, Long>
    ) {
        val (upDuration, downDuration) = durations
        isUp = true
        SleepFor { if (config.isAxon) axonDurationUp else upDuration }
        isUp = false
        SleepFor { if (config.isAxon) axonDurationDown else downDuration }
    }

    fun updateState() {
        with(opMode) {
            v3.gain = config.v3Gain.toFloat()
            val v3Dist = v3.getDistance(DistanceUnit.MM)

            if (v3Dist < distThresV3) {
                // v3 has a detection
                val hsv = v3.normalizedColors.toHSV()
                state = if (hsv.saturation < satThresV3) {
                    State.PURPLE
                } else {
                    State.GREEN
                }

                if (config.useTelemetry) opMode.tel.addData("pod | sensor", 0)
                if (config.useTelemetry) opMode.tel.addData("pod | h", hsv.hue)
                if (config.useTelemetry) opMode.tel.addData("pod | s", hsv.saturation)
                if (config.useTelemetry) opMode.tel.addData("pod | dist", v3Dist)
                return@with
            }
            // v3 is empty
            val v2Dist = v2.getDistance(DistanceUnit.MM)
            if (v2Dist > distThresV2) {
                // both sensors empty
                state = State.EMPTY
                return@with
            }
            // v2 has a detection
            val hsv = v3.normalizedColors.toHSV()
            state = if (hsv.hue > hueThresV2) {
                State.PURPLE
            } else {
                State.GREEN
            }
            if (config.useTelemetry) opMode.tel.addData("pod | sensor", 1)
            if (config.useTelemetry) opMode.tel.addData("pod | h", hsv.hue)
            if (config.useTelemetry) opMode.tel.addData("pod | s", hsv.saturation)
            if (config.useTelemetry) opMode.tel.addData("pod | dist", v2Dist)
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
            "s0", false, .4, .8, "c1e", "c2e"
        )

        @JvmField
        var backConfig = Config(
            "s3", false, .35, .67, "c0", "c0e", isAxon = true
        )

        @JvmField
        var sideConfig = Config(
            "s4", true, .30, .70, "c2", "c1"
        )

        @JvmField
        var axonDurationUp = 500L

        @JvmField
        var axonDurationDown = 100L

        @JvmField
        var distThresV3 = 25.0

        @JvmField
        var distThresV2 = 70.0

        @JvmField
        var satThresV3 = 45.0 // v3 doesn't detect hue properly, only consistent metric

        @JvmField
        var hueThresV2 = 190.0 // seems to be more consistent than saturation for v2
    }

    enum class Slot {
        FRONT, BACK, SIDE
    }

    enum class State {
        EMPTY, PURPLE, GREEN,
    }

    @Configurable
    @Suppress("detekt:LongParameterList")
    class Config(
        var servoName: String,
        var servoReversed: Boolean,
        var downPosition: Double,
        var upPosition: Double,
        var v2Name: String,
        var v3Name: String,
        var v3Gain: Double = 10.0,
        var useTelemetry: Boolean = false,
        var isAxon: Boolean = false
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