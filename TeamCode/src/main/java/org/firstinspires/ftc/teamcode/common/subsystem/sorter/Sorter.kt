package org.firstinspires.ftc.teamcode.common.subsystem.sorter

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.commands.RapidFire.Companion.getFiringSpeed
import org.firstinspires.ftc.teamcode.common.subsystem.IndicatorLight
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class Sorter(
    val opMode: OpMode,
    val indicatorLight: IndicatorLight? = null,
) : Subsystem("Sorter") {
    enum class Pods {
        FRONT,
        BACK,
        SIDE,
    }

    enum class RefreshRate {
        SLOW, // outtaking/idle, intaking w/ gate assist
        MEDIUM, // intaking, intaking w/ zone assist
        FAST, // completely idle intaking
        ;

        fun getPollingRate(): Double =
            when (this) {
                SLOW -> slowPollingRate
                MEDIUM -> mediumPollingRate
                FAST -> fastPollingRate
            }
    }

    var refreshRate = RefreshRate.SLOW

    val frontPod = SorterPod.fromSlot(opMode, SorterPod.Slot.FRONT)
    val sidePod = SorterPod.fromSlot(opMode, SorterPod.Slot.SIDE)
    val backPod = SorterPod.fromSlot(opMode, SorterPod.Slot.BACK)

    val pods =
        mapOf(
            Pods.FRONT to frontPod,
            Pods.SIDE to sidePod,
            Pods.BACK to backPod,
        )

    val greenPod: Pods?
        get() = pods.filter { it.value.state == SorterPod.State.GREEN }.keys.firstOrNull()

    val isFull: Boolean
        get() = pods.values.all { it.state != SorterPod.State.EMPTY }

    override val run: suspend Command.() -> Unit = {
        opMode.scheduler.schedule(
            Command {
                val timer = ElapsedTime()
                OpModeLoop(opMode) {
                    WaitFor { timer.milliseconds() >= refreshRate.getPollingRate() }
                    timer.reset()
                    pods.values.forEach { it.updateState() }
                }
            },
        )
    }

    fun resetKickers() {
        pods.values.forEach { it.isUp = false }
    }

    suspend fun Command.rapidFire(
        firingOrder: List<Pods> = pods.keys.toList(),
        firingSpeed: Pair<Long, Long> = getFiringSpeed(),
    ) {
        firingOrder.forEach {
            val pod = pods[it] ?: return@forEach
            pod.run { kick(firingSpeed) }
        }
    }

    fun getFiringOrder(
        greenPod: Pods,
        greenPosition: Int,
    ): List<Pods> =
        buildList {
            val purples = pods.keys.filter { it != greenPod }
            addAll(purples)
            add(greenPosition, greenPod)
        }

    companion object {
        @JvmField
        var slowPollingRate = 1000.0

        @JvmField
        var mediumPollingRate = 500.0

        @JvmField
        var fastPollingRate = 100.0
    }
}
