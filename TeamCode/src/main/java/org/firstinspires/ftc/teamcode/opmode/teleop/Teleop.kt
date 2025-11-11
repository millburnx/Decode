package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.field.PanelsField
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.TeleopAutoFire
import org.firstinspires.ftc.teamcode.common.subsystem.Apriltags
import org.firstinspires.ftc.teamcode.common.subsystem.AutoTargetting
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Pedro
import org.firstinspires.ftc.teamcode.common.subsystem.Uppies
import org.firstinspires.ftc.teamcode.common.vision.VisionManager
import org.firstinspires.ftc.teamcode.opmode.OpMode

@TeleOp
class Teleop : OpMode() {
    override fun run() {
        val intake = Intake(this, isTeleop = true)
        val flyWheel = FlyWheel(this, isTeleop = true)
        val uppies = Uppies(this, { flyWheel.state }, isTeleop = true)
        val pedro = Pedro(this, isTeleop = true)

        // we can modify this to auto init probably
        // can set a flag on VisionManager.build
        // so subsequent attempts to add to the list will clear it first
        // effectively auto calling VisionManager.init
        VisionManager.init()
        val apriltags = Apriltags()
        VisionManager.build(this)

        val autoTargetting = AutoTargetting(this, pedro, apriltags)

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

                val target = autoTargetting.target
                if (target != null) {
                    field.setStyle("blue", "blue", 2.0)
                    field.moveCursor(target.x, target.y)
                    field.circle(2.0)
                }
                field.update()
                sync()
            }
        })

        scheduler.schedule(TeleopAutoFire(this, intake, flyWheel, uppies))
//        scheduler.schedule(TeleOpStopper(this))
    }
}