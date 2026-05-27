package org.firstinspires.ftc.teamcode.common.subsystem.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.IndicatorLight
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class TeleopSorterManager(
    val opMode: OpMode,
    val sorter: Sorter,
    val indicatorLight: IndicatorLight,
) :
    Subsystem("Teleop Sorter Manager") {
    val sortingConfig: SortingConfig = SortingConfig()
    val greenPod
        get() = sortingConfig.pod ?: sorter.greenPod
    val isSorting
        get() = greenPod != null && sortingConfig.pattern != null

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            with(opMode) {
                // manual sorting
                gp2.a.ifPressed { sortingConfig.reset() }
                gp2.dPad.right.ifPressed { sortingConfig.pod = null }

                gp2.x.ifPressed { sortingConfig.pattern = 0 }
                gp2.y.ifPressed { sortingConfig.pattern = 1 }
                gp2.b.ifPressed { sortingConfig.pattern = 2 }

                gp2.dPad.up.ifPressed { sortingConfig.pod = Sorter.Pods.FRONT }
                gp2.dPad.left.ifPressed { sortingConfig.pod = Sorter.Pods.SIDE }
                gp2.dPad.down.ifPressed { sortingConfig.pod = Sorter.Pods.BACK }

                indicatorLight.stateFlags[IndicatorLight.State.SORTING] = isSorting
            }
        }
    }
}


class SortingConfig(
    var pattern: Int? = null,
    var pod: Sorter.Pods? = null
) {
    fun reset() {
        pattern = null
        pod = null
    }
}