package org.firstinspires.ftc.teamcode.opmode.test.localization

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.PanelsField
import com.millburnx.cmdx.Command
import com.millburnx.util.vector.Vec2d
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

        scheduler.schedule(Command() {
            val canvas = PanelsField.field
            canvas.setOffsets(PanelsField.presets.PEDRO_PATHING)

            OpModeLoop(this@FusionTest) {
                val pose = drive.pose
                canvas.moveCursor(pose.x + 72.0, pose.y + 72.0)
                canvas.setStyle(fill = "none", outline = "white", width = 1.5)
                canvas.circle(8.0)
                val lookPos = pose + Vec2d(72.0, 72.0) + Vec2d(1.0, 0.0).rotate(pose.radians) * 8.0
                canvas.line(lookPos.x, lookPos.y)

                tel.addData("pose", pose)
                tel.addData("pose.x", pose.x)
                tel.addData("pose.y", pose.y)
                tel.addData("pose.h", pose.heading)

                val llPose = limelight.pose
                if (llPose != null) {
                    canvas.moveCursor(llPose.first.x + 72.0, llPose.first.y + 72.0)
                    canvas.setStyle(fill = "none", outline = "green", width = 1.5)
                    canvas.circle(8.0)
                    val llLookPos = llPose.first + Vec2d(72.0, 72.0) + Vec2d(1.0, 0.0).rotate(pose.radians) * 8.0
                    canvas.line(llLookPos.x, llLookPos.y)

                    tel.addData("llpose", llPose.first)
                    tel.addData("llpose.x", llPose.first.x)
                    tel.addData("llpose.y", llPose.first.y)
                }
            }
        })
    }
}