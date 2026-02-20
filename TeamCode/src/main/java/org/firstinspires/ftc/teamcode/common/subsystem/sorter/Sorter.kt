package org.firstinspires.ftc.teamcode.common.subsystem.sorter

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class Sorter(val opMode: OpMode) : Subsystem("FlyWheel") {
    val frontPod = SorterPod.fromSlot(opMode, SorterPod.Slot.FRONT)
    val backPod = SorterPod.fromSlot(opMode, SorterPod.Slot.BACK)
    val sidePod = SorterPod.fromSlot(opMode, SorterPod.Slot.SIDE)

    override val run: suspend Command.() -> Unit = {
//        opMode.scheduler.schedule(Command {
//            OpModeLoop(opMode) {
//                SleepFor { colorPolling }
//                frontPod.updateState()
//                sidePod.updateState()
//                backPod.updateState()
//            }
//        })
        OpModeLoop(opMode) {
            with(opMode) {
//                tel.addData("front state", frontPod.state)
//                tel.addData("back state", backPod.state)
//                tel.addData("side state", sidePod.state)
            }
        }
    }

    companion object {
        @JvmField
        var colorPolling = 2000L
    }
}