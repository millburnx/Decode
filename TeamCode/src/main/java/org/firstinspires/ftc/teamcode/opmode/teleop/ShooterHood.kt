package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Hood
import org.firstinspires.ftc.teamcode.common.subsystem.Kickers
import org.firstinspires.ftc.teamcode.common.subsystem.Limelight
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class ShooterHood : OpMode() {
    override fun run() {
        val drive = Drive(opMode = this, isTeleop = true)
        val kickers = Kickers(this, isTeleop = true)
        val flyWheel = FlyWheel(this, isTeleop = true)
        val hood = Hood(this, isTeleop = true)
        val ll = Limelight(this) { drive.pose.heading }



        scheduler.schedule(Command("rapid fire scheduler") {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                flyWheel.enabled = true
                flyWheel.targetRpm = rpmTarget
                hood.position = hoodTarget

                val llPose = ll.llPose
                if (llPose != null) {
                    val goal = Vec2d(12.0, 144.0 - 12.0)
                    tel.addData("ll dist", llPose.distanceTo(goal))
                }
                sync()
            }
        })
    }

    companion object {
        @JvmField
        var rpmTarget = 0.0

        @JvmField
        var hoodTarget = 0.0
    }
}