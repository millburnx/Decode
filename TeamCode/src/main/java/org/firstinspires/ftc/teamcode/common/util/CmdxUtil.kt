package org.firstinspires.ftc.teamcode.common.util

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.opmode.OpMode

class OpModeLoopExit : RuntimeException(null, null, false, false)

suspend fun Command.OpModeLoop(opMode: OpMode, runnable: suspend Command.() -> Unit) {
    with(opMode) {
        WaitFor { isStarted || isStopRequested }
        while (!isStopRequested) {
            try {
                runnable()
            } catch (_: OpModeLoopExit) {
                break
            }
            sync()
        }
    }
}