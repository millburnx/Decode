package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.common.subsystem.Hood
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
        val hood = Hood(this, isTeleop = true)
        scheduler.schedule(Command() {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                val llPose = limelight.llPose
                if (override) {
                    turret.targetingMode =
                        if (overrideGlobal) Turret.TargetingMode.GLOBAL else Turret.TargetingMode.RELATIVE
                    turret.targetAngle = overrideAngle
                    hood.position = overrideHood
                } else if (llPose == null) {
                    turret.targetingMode = Turret.TargetingMode.RELATIVE
                    turret.targetAngle = 0.0
                } else {
                    val goal = Vec2d(12.0, 144.0 - 12.0)
                    val angle = llPose.position.angleTo(goal).toDegrees()
                    hood.position = Hood.min + (llPose.position.distance(goal) / 144.0) * (Hood.max - Hood.min)
                    tel.addData("vision | target angle", angle)
                    turret.targetingMode = Turret.TargetingMode.GLOBAL
                    turret.targetAngle = angle
                }
                sync()
            }
        })
    }

    companion object {
        @JvmField
        var override = false

        @JvmField
        var overrideGlobal = false

        @JvmField
        var overrideAngle = 90.0

        @JvmStatic
        var overrideHood = 1.0
    }
}