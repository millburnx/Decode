package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Parallel
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Autonomous
class Close18BallRedWLR : Close18BallAutonWLR(true)

@Autonomous
class Close18BallBlueWLR : Close18BallAutonWLR(false)

@Configurable
open class Close18BallAutonWLR(var isRed: Boolean) : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = Drive(this)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        val autoAdjust = AutoAdjust(this, flyWheel, hood, { drive.pose }, { Vec2d() }, isRed)

        val autonManager = AutonManager(this, drive, "Close-18Ball-RampAndRowsTangent", isMirrored = !isRed)

        val fire = Command {
            intake.power = -1.0
            WaitFor { turret.atTarget && turret.isSteady }
            val atRPM = { flyWheel.atRPM }
            WaitFor { atRPM() }
            sorter.run { rapidFire(firingSpeed = upDuration to downDuration) }
            intake.power = 1.0
        }

        val gateIntake = Sequential {
            Command {
                drive.follower.breakFollowing()
                drive.follower.startTeleopDrive(true)
                drive.follower.setTeleOpDrive(gatePower, 0.0, gateCounterRotate, false)

                SleepFor { gateDuration }

                drive.follower.setTeleOpDrive(0.0, 0.0, 0.0)

                SleepFor { rampDuration }
            }
        }

        val gateCycle = Sequential {
            +autonManager.runPath(4)
            +gateIntake
            +Parallel {
                +autonManager.runPath(6)
                Command {
                    SleepFor { 100 }
                    WaitFor { drive.follower.currentTValue >= rampOutakeT }
                    intake.power = -1.0
                }
            }
        }

        val endAuton = Command {
            intake.power = 0.0
            FlyWheel.override = true
            FlyWheel.overridePower = 0.0
            println("auton end ${matchTimer.seconds()}")
        }

        val goal = if (autonManager.isMirrored) Vec2d(-3.0, 144.0) else Vec2d(144.0, 144.0)

        scheduler.schedule(Command("general") {
            OpModeLoop(this@Close18BallAutonWLR) {
                turret.targetingMode = Turret.TargetingMode.GLOBAL
                turret.target = drive.pose.position.angleTo(goal).toDegrees()
            }
        })

        scheduler.schedule(Sequential("Close Auton Safe") {
            Command("Start") {
                WaitFor { isStarted }
                flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
            }
            +autonManager.runPath(0) // preload
            +fire
            +autonManager.runPath(1) // row 2
            +autonManager.runPath(2)
            +Parallel {
                +autonManager.runPath(3)
                Command {
                    SleepFor { 100 }
                    WaitFor { drive.follower.currentTValue >= row2OuttakeT }
                    intake.power = -1.0
                }
            }
            +fire
            repeat(3) { // ramp
                +gateCycle
                +fire
            }
            +Parallel {
                +autonManager.runPath(10) // row 1
                Command {
                    SleepFor { 100 }
                    WaitFor { drive.follower.currentTValue >= row1SlowT }
                    drive.follower.setMaxPower(row1SlowSpeed)
                }
            }
            +Parallel {
                +autonManager.runPath(11)
                Command {
                    drive.follower.setMaxPower(1.0)
                    SleepFor { 100 }
                    WaitFor { drive.follower.currentTValue >= row1OuttakeT }
                    intake.power = -1.0
                }
            }
            +fire
            +endAuton
        })
    }

    companion object {
        @JvmField
        var upDuration = 175L

        @JvmField
        var downDuration = 50L

        @JvmField
        var gatePower = 0.6

        @JvmField
        var gateCounterRotate = -0.15

        @JvmField
        var gateDuration = 500L

        @JvmField
        var rampDuration = 1500L

        @JvmField
        var rampOutakeT = 0.125

        @JvmField
        var row2OuttakeT = 0.75

        @JvmField
        var row2SlowT = 0.25

        @JvmField
        var row2SlowSpeed = 0.75

        @JvmField
        var row3OuttakeT = 0.375

        @JvmField
        var row1OuttakeT = 0.75

        @JvmField
        var row1SlowT = 0.25

        @JvmField
        var row1SlowSpeed = 0.75
    }
}