package org.firstinspires.ftc.teamcode.common.subsystem.sorter

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop

@Configurable
class Sorter(val opMode: OpMode, val indicatorLight: IndicatorLight? = null) : Subsystem("Sorter") {
    enum class Pods {
        FRONT, BACK, SIDE
    }

    val frontPod = SorterPod.fromSlot(opMode, SorterPod.Slot.FRONT)
    val sidePod = SorterPod.fromSlot(opMode, SorterPod.Slot.SIDE)
    val backPod = SorterPod.fromSlot(opMode, SorterPod.Slot.BACK)

    val pods = mapOf(
        Pods.FRONT to frontPod,
        Pods.SIDE to sidePod,
        Pods.BACK to backPod,
    )

    override val run: suspend Command.() -> Unit = {
        opMode.scheduler.schedule(Command {
            val timer = ElapsedTime()
            OpModeLoop(opMode) {
                WaitFor { timer.milliseconds() >= pollingRate }
                timer.reset()
                pods.values.forEach { it.updateState() }
//                indicatorLight?.let {
//                    val nonEmpty = pods.values.filter { it.state != SorterPod.State.EMPTY }
//                }
            }
        })
    }

    fun resetKickers() {
        pods.values.forEach { it.isUp = false }
    }

    suspend fun Command.rapidFire(
        firingOrder: List<Pods> = pods.keys.toList(),
        firingSpeed: Pair<Long, Long> = Teleop.getFiringSpeed()
    ) {
        firingOrder.forEach {
            val pod = pods[it] ?: return@forEach
            pod.run { kick(firingSpeed) }
        }
    }

    fun getFiringOrder(greenPod: Pods, greenPosition: Int): List<Pods> = buildList {
        val purples = pods.keys.filter { it != greenPod }
        addAll(purples)
        add(greenPosition, greenPod)
    }

    companion object {
        @JvmField
        var pollingRate = 5000.0
    }
}