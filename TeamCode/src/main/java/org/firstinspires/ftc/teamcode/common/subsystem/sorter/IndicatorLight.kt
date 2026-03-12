package org.firstinspires.ftc.teamcode.common.subsystem.sorter

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualServo
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class IndicatorLight(val opMode: OpMode) : Subsystem("IndicatorLight") {
    val indicatorLight = ManualServo(opMode.hardwareMap, indicatorLightName)

    var stateFlags: MutableMap<State, Boolean> = mutableMapOf(
        State.SORTING to false,
        State.OUT_OF_ZONE to false,
        State.TURRET_DEADZONE to false,
        State.FIRING to false
    )

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            OpModeLoop(opMode) {
                val primaryState = stateFlags.entries.reversed().firstOrNull { it.value }?.key ?: State.IDLE
                indicatorLight.position = getColor(primaryState)
                tel.addData("Indicator State", primaryState.name)
            }
        }
    }

    enum class State {
        IDLE,
        SORTING,
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
        var outOfZoneColor = .36

        @JvmField
        var sortingColor = .66

        @JvmField
        var turretDeadzoneColor = .28

        @JvmField
        var firingColor = .5

        fun getColor(state: State): Double {
            return when (state) {
                State.IDLE -> idleColor
                State.OUT_OF_ZONE -> outOfZoneColor
                State.SORTING -> sortingColor
                State.TURRET_DEADZONE -> turretDeadzoneColor
                State.FIRING -> firingColor
            }
        }
    }
}