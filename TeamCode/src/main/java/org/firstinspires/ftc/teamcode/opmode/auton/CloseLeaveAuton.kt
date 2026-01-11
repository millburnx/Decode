package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
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
class CloseLeaveAuton : OpMode() {
    override fun run() {
        val autonManager = AutonManager(this, "closeLeaveAuton")
        val drive = Drive(this, isTeleop = false, follower = autonManager.follower)
        val intake = Intake(this, isTeleop = false)
        val kickers = Kickers(this, isTeleop = false)
//        val turret = Turret(this, isTeleop = false) { drive.pose }
        val flyWheel = FlyWheel(this, isTeleop = false)
        val hood = Hood(this, isTeleop = false)
//        val limelight = Limelight(this) { turret.globalAngle }

        val goal = Vec2d(12.0, 144.0 - 12.0)
        with(autonManager) {
            scheduler.schedule(Sequential("Close Auton") {
                Command("Start") {
                    WaitFor { isStarted }
                }
                Command("Init") {
                    println("enabled flywheel")
//                    turret.targetingMode = Turret.TargetingMode.GLOBAL
//                    turret.targetAngle = 145.0
//                    flyWheel.enabled = true
                    flyWheel.targetRpm = 1900.0
                    hood.position = 0.25
                }
                +runPath(0)
                Command("Settling") {
                    SleepFor { settling }
                }
//                +kickers.rapidFire
                Command("disable flywheel") {
                    println("disabled flywheel")
                    flyWheel.enabled = false
                }
                +runPath(1)
                Command("enable intake") {
                    intake.power = 1.0
                    follower.setMaxPower(intakeSpeed)
                }
                +runPath(2)
                Command("disable intake") {
                    SleepFor { intakeDuration }
                    follower.setMaxPower(1.0)
                    intake.power = 0.0
//                    flyWheel.enabled = true
                    flyWheel.targetRpm = 1900.0
                    hood.position = 0.25
                }
                +runPath(3)
//                +kickers.rapidFire
                Command {
                    flyWheel.enabled = false
                }
            })
        }
    }

    companion object {
        @JvmField
        var settling = 1000L

        @JvmField
        var intakeDuration = 1000L

        @JvmField
        var intakeSpeed = 0.2
    }
}