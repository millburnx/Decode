package org.firstinspires.ftc.teamcode.opmode.teleop

import org.firstinspires.ftc.teamcode.common.commands.TeleOpStopper
import org.firstinspires.ftc.teamcode.opmode.OpMode

class SampleTeleop: OpMode() {
    override fun run() {
        scheduler.schedule(TeleOpStopper(this))
    }
}