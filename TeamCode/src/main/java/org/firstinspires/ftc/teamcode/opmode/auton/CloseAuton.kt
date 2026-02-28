package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Configurable
@Autonomous
class CloseAuton : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = Drive(this)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        val autoAdjust = AutoAdjust(this, flyWheel, hood, { drive.pose })

        val autonManager = AutonManager(this, drive, "closeauton", isMirrored = true)

        val fire = Command {
            intake.power = -1.0

            WaitFor { turret.atTarget && turret.isSteady }

            val minRPM = { autoAdjust.minRapidRPM - FlyWheel.Controller.rpmThreshold }
            val maxRPM = { flyWheel.shootingRPM + FlyWheel.Controller.rpmThreshold }

            val atRPM = { flyWheel.rpm in minRPM()..maxRPM() }

            WaitFor { atRPM() }
            sorter.frontPod.isUp = true
            SleepFor { upDuration }
            sorter.frontPod.isUp = false
            SleepFor { downDuration }

            WaitFor { atRPM() }
            sorter.sidePod.isUp = true
            SleepFor { upDuration }
            sorter.sidePod.isUp = false
            SleepFor { downDuration }

            WaitFor { atRPM() }
            sorter.backPod.isUp = true
            SleepFor { upDuration }
            sorter.backPod.isUp = false
            SleepFor { downDuration }

            intake.power = 1.0
        }

        val gateCycle = Sequential("gate cycle") {
            Command {
                intake.power = 1.0
            }
            +autonManager.runPath(5,)
            Command {
                SleepFor { intakeTime }
                intake.power = -1.0
            }
            +autonManager.runPath(6)
            Command {
                SleepFor { stablizationTime }
            }
            +fire
        }

        val goal = if (autonManager.isMirrored) Vec2d(0.0, 144.0) else Vec2d(140.0, 140.0)

        scheduler.schedule(Command("general") {
            while (!isStopRequested) {
                if (isStarted) {
                    val isBusy = drive.follower.isBusy
                    val isTurning = drive.follower.isTurning
                    val isRobotStuck = drive.follower.isRobotStuck
                    val currentTValue =
                        if (drive.follower.currentPath != null) drive.follower.currentTValue else -1.0
                    val atParametricEnd =
                        if (drive.follower.currentPath != null) drive.follower.atParametricEnd() else -1.0
                    println("isBusy $isBusy | isTurning $isTurning | isRobotStuck $isRobotStuck | currentTValue $currentTValue | atParametricEnd $atParametricEnd")
                }
                sync()
            }
        })

        scheduler.schedule(Sequential("Close Auton") {
            Command("Start") {
                WaitFor { isStarted }
            }
            Command {
                flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
                turret.targetingMode = Turret.TargetingMode.GLOBAL
                turret.target = 45.0.mirror(autonManager.isMirrored)
            }
            +autonManager.runPath(0)
            Command { SleepFor { stablizationTime } }
            +fire
            Command { intake.power = 1.0 }
            +autonManager.runPath(3)
            +autonManager.runPath(4)
            Command { SleepFor { stablizationTime } }
            +fire
            +gateCycle
            Command { intake.power = 1.0 }
            +autonManager.runPath(1)
            +autonManager.runPath(2)
            Command { SleepFor { stablizationTime } }
            +fire
            +autonManager.runPath(7)
            Command {
                turret.target =
                    normalizeDegrees(
                        Vec2d(102, 13).mirror(autonManager.isMirrored)
                            .angleTo(goal)
                            .toDegrees()
                    )
                autoAdjust.forceFar = true
            }
            +autonManager.runPath(8)
            Command { SleepFor { stablizationTime } }
            Command {
                turret.target =
                    normalizeDegrees(
                        drive.pose
                            .angleTo(goal)
                            .toDegrees()
                    )
            }
            +fire
            Command { println("auton end ${matchTimer.seconds()}") }
        })
    }

    companion object {
        @JvmField
        var upDuration = 200L

        @JvmField
        var downDuration = 50L

        @JvmField
        var intakeTime = 2500L

        @JvmField
        var stablizationTime = 300L
    }
}