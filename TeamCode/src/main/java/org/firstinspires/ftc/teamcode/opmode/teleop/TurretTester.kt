package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Turret
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp

class Teleop : OpMode() {
    override fun run() {
        val turret = Turret(this, isTeleop = true)
    }
}