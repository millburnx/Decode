package org.firstinspires.ftc.teamcode.opmode.test.localization

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.PanelsField
import com.millburnx.cmdx.Command
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleOpDrive
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Disabled
@Configurable
@TeleOp(group = "test")
class PinpointTest : OpMode() {
    override fun run() {
        val drive = TeleOpDrive(this, true)

        scheduler.schedule(Command() {
            val canvas = PanelsField.field
            canvas.setOffsets(PanelsField.presets.PEDRO_PATHING)

            OpModeLoop(this@PinpointTest) {
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
            }
        })
    }
}