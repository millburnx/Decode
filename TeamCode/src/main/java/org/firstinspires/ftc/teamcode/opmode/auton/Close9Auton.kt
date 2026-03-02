package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Autonomous
class Close9AutonRed : Close9Auton(true)

@Autonomous
class Close9AutonBlue : Close9Auton(false)

@Configurable
open class Close9Auton(var isRed: Boolean) : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = Drive(this)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        val autoAdjust = AutoAdjust(this, flyWheel, hood, { drive.pose }, isRed)

        val autonManager = AutonManager(this, drive, "closeautonsafe", isMirrored = !isRed)

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

//            WaitFor { atRPM() }
            sorter.sidePod.isUp = true
            SleepFor { upDuration }
            sorter.sidePod.isUp = false
            SleepFor { downDuration }

//            WaitFor { atRPM() }
            sorter.backPod.isUp = true
            SleepFor { upDuration }
            sorter.backPod.isUp = false
            SleepFor { downDuration }

            intake.power = 1.0
        }

        val goal = if (autonManager.isMirrored) Vec2d(0.0, 144.0) else Vec2d(144.0, 144.0)

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

                    GlobalStore.autonPose = drive.pose
                }
                sync()
            }
        })

        scheduler.schedule(Sequential("Close Auton Safe") {
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
            +autonManager.runPath(1)
            +autonManager.runPath(2)
            Command { SleepFor { stablizationTime } }
            +fire
            +autonManager.runPath(3)
            +autonManager.runPath(4)
            Command { SleepFor { stablizationTime } }
            +fire
            +autonManager.runPath(7)
            Command { println("auton end ${matchTimer.seconds()}") }
        })
    }

    companion object {
        @JvmField
        var upDuration = 150L

        @JvmField
        var downDuration = 50L

        @JvmField
        var stablizationTime = 300L

        @JvmField
        var intakePower = .8
    }
}