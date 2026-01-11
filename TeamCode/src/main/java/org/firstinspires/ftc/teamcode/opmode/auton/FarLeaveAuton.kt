package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Hood
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Kickers
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Configurable
@Autonomous
class FarLeaveAuton : OpMode() {
    override fun run() {
        val autonManager = AutonManager(this, "farAuton")
        val drive = Drive(this, isTeleop = false, follower = autonManager.follower)
        val intake = Intake(this, isTeleop = false)
        val kickers = Kickers(this, isTeleop = false)
//        val turret = Turret(this, isTeleop = false) { drive.pose }
        val flyWheel = FlyWheel(this, isTeleop = false)
        val hood = Hood(this, isTeleop = false)
//        val limelight = Limelight(this) { turret.globalAngle }

        val goal = Vec2d(12.0, 144.0 - 12.0)
        scheduler.schedule(Sequential("Close Auton") {
            Command("Start") {
                WaitFor { isStarted }
            }
            +autonManager.runPath(0)
        })
    }
}