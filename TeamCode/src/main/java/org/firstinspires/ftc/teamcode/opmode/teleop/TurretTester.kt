package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.common.subsystem.Limelight
import org.firstinspires.ftc.teamcode.common.subsystem.Turret
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp

class TurretTester : OpMode() {
    override fun run() {
        val drive = Drive(this, isTeleop = true, Pose2d(72.0, 72.0, 0.0))
        val turret = Turret(this, isTeleop = true) { drive.pose }
        val limelight = Limelight(this, turret)
        scheduler.schedule(Command() {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
//                turret.targetAngle = targetAngle
                val llPose = limelight.llPose
                if (override) {
                    turret.enabled = true
                    turret.targetAngle = targetAngle
                } else if (llPose == null) {
                    turret.enabled = false
                } else {
                    turret.enabled = true
                    val angle = llPose.position.angleTo(Vec2d(12.0, 144.0 - 12.0)).toDegrees()
                    tel.addData("vision | target angle", angle)
                    turret.targetAngle = (-angle + 360) % 360
                }
                sync()
            }
        })
    }

    companion object {
        @JvmField
        var override = false
        @JvmField
        var targetAngle = 180.0
    }
}