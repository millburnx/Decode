package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.common.subsystem.Limelight
import org.firstinspires.ftc.teamcode.common.subsystem.Turret
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp

class TurretTester : OpMode() {
    override fun run() {
        val drive = Drive(this, isTeleop = true)
        val turret = Turret(this, isTeleop = true) { drive.pose }
        val limelight = Limelight(this, turret)
        scheduler.schedule(Command() {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                turret.targetAngle = targetAngle
                sync()
            }
        })
    }

    companion object {
        @JvmField
        var targetAngle = 180.0
    }
}