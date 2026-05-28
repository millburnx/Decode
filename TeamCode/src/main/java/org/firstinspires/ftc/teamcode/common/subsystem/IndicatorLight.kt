package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualServo
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class IndicatorLight(
    val opMode: OpMode,
) : Subsystem("IndicatorLight") {
    val light = ManualServo(opMode.hardwareMap, indicatorLightName)

    var stateFlags: MutableMap<State, Boolean> =
        mutableMapOf(
            State.SORTING to false,
            State.FULL to false,
            State.OUT_OF_ZONE to false,
            State.TURRET_DEADZONE to false,
            State.FIRING to false,
        )

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            OpModeLoop(opMode) {
                val primaryState =
                    stateFlags.entries
                        .reversed()
                        .firstOrNull { it.value }
                        ?.key ?: State.IDLE
                light.position = getColor(primaryState)
                tel.addData("Indicator State", primaryState.name)
            }
        }
    }

    fun fullFlash() {
        if (stateFlags[State.FULL] == true) return // don't have two timers running

        opMode.scheduler.schedule(
            Command("full flash") {
                stateFlags[State.FULL] = true
                SleepFor { fullFlashDuration }
                stateFlags[State.FULL] = false
            },
        )
    }

    enum class State {
        IDLE,
        SORTING,
        FULL,
        OUT_OF_ZONE,
        TURRET_DEADZONE,
        FIRING,
    }

    companion object {
        @JvmField
        var indicatorLightName = "s1"

        @JvmField
        var idleColor = 1.0

        @JvmField
        var sortingColor = .66

        @JvmField
        var fullColor = 0.6

        @JvmField
        var outOfZoneColor = .36

        @JvmField
        var turretDeadzoneColor = .28

        @JvmField
        var firingColor = .5

        @JvmField
        var fullFlashDuration = 500L

        fun getColor(state: State): Double =
            when (state) {
                State.IDLE -> idleColor
                State.SORTING -> sortingColor
                State.FULL -> fullColor
                State.OUT_OF_ZONE -> outOfZoneColor
                State.TURRET_DEADZONE -> turretDeadzoneColor
                State.FIRING -> firingColor
            }
    }
}
