package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.TeleOpStopper
import org.firstinspires.ftc.teamcode.opmode.OpMode

@TeleOp
class SampleTeleop: OpMode() {
    override fun run() {
        scheduler.schedule(TeleOpStopper(this))
    }
}