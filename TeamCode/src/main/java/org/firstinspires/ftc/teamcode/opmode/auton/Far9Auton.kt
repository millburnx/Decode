package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.normalize
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Autonomous
class Far9AutonRed : Far9Auton(true)

@Autonomous
class Far9AutonBlue : Far9Auton(false)

@Configurable
open class Far9Auton(var isRed: Boolean) : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = Drive(this)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        val autoAdjust = AutoAdjust(this, flyWheel, hood, { drive.pose }, { Vec2d() }, isRed)

        val autonManager = AutonManager(this, drive, "farauton", isMirrored = !isRed)

        val goal = if (autonManager.isMirrored) Vec2d(0.0, 144.0) else Vec2d(144.0, 144.0)

        val fire = Command {
            intake.power = -1.0

            WaitFor { turret.atTarget && turret.isSteady }

            turret.target = drive.pose.angleTo(goal).toDegrees().normalize() - angleOffset

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
//                autoAdjust.forceFar = true
                turret.targetingMode = Turret.TargetingMode.GLOBAL
                turret.target = drive.pose.angleTo(goal).toDegrees().normalize() - angleOffset
            }
            +autonManager.runPath(0)
            +fire
            Command {
                intake.power = 1.0
            }
            +autonManager.runPath(1) { builder ->
                builder.addParametricCallback(.5, { drive.follower.setMaxPower(intakePower) })
            }
            Command {
                drive.follower.setMaxPower(1.0)
                SleepFor { intakeDuration }
            }
            +autonManager.runPath(2)
            Command { SleepFor { stablizationTime } }
            +fire
            +autonManager.runPath(3)
            +autonManager.runPath(4)
            Command { SleepFor { stablizationTime } }
            +fire
            +autonManager.runPath(5)
            Command { println("auton end ${matchTimer.seconds()}") }
        })
    }

    companion object {
        @JvmField
        var upDuration = 150L

        @JvmField
        var downDuration = 50L

        @JvmField
        var stablizationTime = 2000L

        @JvmField
        var intakePower = .8

        @JvmField
        var intakeDuration = 5000L

        @JvmField
        var angleOffset = -2.0
    }
}