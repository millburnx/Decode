package org.firstinspires.ftc.teamcode.common.util

import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.opmode.OpMode

class DeltaTime(opMode: OpMode) : Subsystem("Delta Time") {
    val timer = ElapsedTime()
    var dt = 0.0

    override val run: suspend Command.() -> Unit = {
        timer.reset()
        while (!opMode.isStopRequested) {
            dt = timer.seconds()
            timer.reset()
            sync()
        }
    }
}