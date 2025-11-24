package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.field.PanelsField
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Pedro
import org.firstinspires.ftc.teamcode.common.vision.Limelight
import org.firstinspires.ftc.teamcode.opmode.OpMode

@TeleOp
class TeleopLL : OpMode() {
    override fun run() {
        val pedro = Pedro(this, Pose2d(72.0, 72.0, 90.0), isTeleop = true)
        val ll = Limelight(this, pedro)

        scheduler.schedule(Command("drawer") {
            while (!isStopRequested) {
                val field = PanelsField.field
                field.setOffsets(PanelsField.presets.PEDRO_PATHING)
                field.setStyle("transparent", "red", 2.0)
                field.moveCursor(pedro.pose.x, pedro.pose.y)
                field.circle(7.0)
                field.setFill("red")
                val lookVector = pedro.pose.position + Vec2d(7.0).rotate(pedro.pose.radians)
                field.line(lookVector.x, lookVector.y)
                field.update()
                sync()
            }
        })
    }
}