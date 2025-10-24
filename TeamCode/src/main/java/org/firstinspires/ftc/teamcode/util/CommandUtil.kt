package org.firstinspires.ftc.teamcode.util

import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.util.ElapsedTime

suspend fun Command.WaitFor(case: () -> Boolean) {
    while (!case()) {
        sync()
    }
}

suspend fun Command.SleepFor(ms: Long) {
    val elapsedTime = ElapsedTime()
    WaitFor { elapsedTime.milliseconds() >= ms }
}