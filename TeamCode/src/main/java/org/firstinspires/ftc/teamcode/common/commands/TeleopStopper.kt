package org.firstinspires.ftc.teamcode.common.commands

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.opmode.OpMode

// Automatically ends teleop after 2 minutes
fun TeleOpStopper(opMode: OpMode) = Command("TeleOp Stopper") {
    WaitFor { opMode.isStarted() || opMode.isStopRequested }
    SleepFor(120 * 1000)
    opMode.requestOpModeStop()
}