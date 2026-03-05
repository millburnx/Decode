package org.firstinspires.ftc.teamcode.opmode.test.localization

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.util.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Limelight
import org.firstinspires.ftc.teamcode.common.subsystem.TeleOpDrive
import org.firstinspires.ftc.teamcode.common.subsystem.Turret
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class FusionTest : OpMode() {
    override fun run() {
        val limelight = Limelight(this)
        val drive = TeleOpDrive(this, true, limelight)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        limelight.turretHeading = { turret.angle }
        limelight.drawPose = { pose ->
            drive.drawRobot(pose, "llPose", "yellow")
        }

        scheduler.schedule(Command {
            OpModeLoop(this@FusionTest) {
                if (gp1.prev.leftBumper && gp1.current.leftBumper) {
                    drive.pose = Pose2d(72.0, 72.0, 0.0)
                }
            }
        })
    }
}