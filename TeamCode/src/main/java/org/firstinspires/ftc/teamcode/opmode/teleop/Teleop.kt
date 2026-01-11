package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Hood
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Kickers
import org.firstinspires.ftc.teamcode.common.subsystem.Limelight
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp

class Teleop : OpMode() {
    override fun run() {
        val drive = Drive(this, isTeleop = true, Pose2d(72.0, 72.0, 0.0))
        val intake = Intake(this, isTeleop = true)
//        val turret = Turret(this, isTeleop = true) { drive.pose }
        val flyWheel = FlyWheel(this, isTeleop = true)
        val kickers = Kickers(this, isTeleop = true) { flyWheel.atVelocity }
        val hood = Hood(this, isTeleop = true)
        val limelight = Limelight(this) { drive.pose.heading }

        val rapidFire = Sequential("rapid fire") {
            Command("start flywheel") {
                flyWheel.enabled = true
//                SleepFor { FlyWheel.spinUpDuration }
            }
            +kickers.rapidFire
            Command("stop flywheel") {
                flyWheel.enabled = false
            }
        }

        var turretDisabled = true

        scheduler.schedule(Command("rapid fire scheduler") {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (gp1.current.rightBumper && !gp1.prev.rightBumper) {
                    scheduler.schedule(rapidFire)
                }
                if (gp1.current.dPad.down && !gp1.prev.dPad.down) {
                    turretDisabled = !turretDisabled
                }
                if (gp1.current.leftBumper && !gp1.prev.leftBumper) {
                    if (flyWheel.targetRpm == FlyWheel.intakeRPM && flyWheel.enabled) {
                        println("disable reverse")
                        flyWheel.enabled = false
                    } else {
                        println("reverse")
                        flyWheel.enabled = true
                        flyWheel.targetRpm = FlyWheel.intakeRPM
                    }
                }
                sync()
            }
        })

        scheduler.schedule(Command("tracker updater") {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                val goal = Vec2d(6.0, 144.0 - 6.0)
                var hasRelocalized = false

                val llPose = limelight.llPose
                if (llPose != null && llPose.x < 144.0 && llPose.x > 0.0 && llPose.y < 144.0 && llPose.y > 0.0) {
                    require(llPose != null)
                    drive.follower.pose = llPose.toPedro()
                    hasRelocalized = true
                }
                if (flyWheel.targetRpm == FlyWheel.intakeRPM) {
//                    if (!turretDisabled) {
//                        turret.targetingMode = Turret.TargetingMode.RELATIVE
//                        turret.targetAngle = 0.0
//                    } else {
//                        turret.targetingMode = Turret.TargetingMode.OFF
//                    }
                    hood.position = 0.0
                } else if (hasRelocalized) {
                    val angle = drive.pose.position.angleTo(goal).toDegrees()
//                    if (!turretDisabled) {
//                        turret.targetingMode = Turret.TargetingMode.GLOBAL
//                        turret.targetAngle = angle
//                    } else {
//                        turret.targetingMode = Turret.TargetingMode.OFF
//                    }

                    val distance = drive.pose.position.distance(goal)
                    val settings = FlyWheel.getSettings(distance)
                    flyWheel.targetRpm = settings.first
                    hood.position = settings.second

                    tel.addData("vision | target angle", angle)
                    tel.addData("vision | target distance", distance)
                } else {
                    flyWheel.targetRpm = 1900.0
                    hood.position = .7
//                    if (!turretDisabled) {
//                        turret.targetingMode = Turret.TargetingMode.RELATIVE
//                        turret.targetAngle = 0.0
//                    } else {
//                        turret.targetingMode = Turret.TargetingMode.OFF
//                    }
                }
                sync()
            }
        })
    }
}