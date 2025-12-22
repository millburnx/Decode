package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.common.subsystem.Hood
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Kickers
import org.firstinspires.ftc.teamcode.common.subsystem.Shooter
import org.firstinspires.ftc.teamcode.common.subsystem.Turret
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp

class Teleop : OpMode() {
    override fun run() {
        val drive = Drive(this, isTeleop = true)

        val intake = Intake(this, isTeleop = true)

        val kickers = Kickers(this, isTeleop = true)

        val turret = Turret(this, isTeleop = true)
        val shooter = Shooter(this, isTeleop = true)
        val hood = Hood(this, isTeleop = true)
    }
}