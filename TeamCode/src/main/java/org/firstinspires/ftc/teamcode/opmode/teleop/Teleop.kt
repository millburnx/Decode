package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.TeleopAutoFire
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Pedro
import org.firstinspires.ftc.teamcode.common.subsystem.Uppies
import org.firstinspires.ftc.teamcode.opmode.OpMode

@TeleOp
class Teleop : OpMode() {
    override fun run() {
        val intake = Intake(this, isTeleop = true)
        val flyWheel = FlyWheel(this, isTeleop = true)
        val uppies = Uppies(this, { flyWheel.state }, isTeleop = true)
        val pedro = Pedro(this, isTeleop = true)

        scheduler.schedule(TeleopAutoFire(this, intake, flyWheel, uppies))
//        scheduler.schedule(TeleOpStopper(this))
    }
}