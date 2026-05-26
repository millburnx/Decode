package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Parallel
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Autonomous(preselectTeleOp = "TeleopRed")
class Close18BallRed : Close18BallAuton(true)

@Autonomous(preselectTeleOp = "TeleopBlue")
class Close18BallBlue : Close18BallAuton(false)

@Configurable
open class Close18BallAuton(var isRed: Boolean) : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = Drive(this)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        val autoAdjust = AutoAdjust(this, flyWheel, hood, drive, isRed)

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
                drive.follower.setTeleOpDrive(
                    gatePower * (if (isRed) 1.0 else -1.0),
                    0.0,
                    gateCounterRotate * (if (isRed) 1.0 else -1.0), false
                )

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
                    ParametricCallback(drive.follower, 0, rampOutakeT) {
                        intake.power = -1.0
                    }
                }

            }
        }

        val endAuton = Command {
            intake.power = 0.0
            FlyWheel.override = true
            FlyWheel.overridePower = 0.0
            println("auton end ${matchTimer.seconds()}")
        }

        val goal = Vec2d(-4.0, 144.0).mirror(isRed)

        scheduler.schedule(Command("general") {
            OpModeLoop(this@Close18BallAuton) {
                turret.targetingMode = Turret.TargetingMode.GLOBAL
                turret.target = drive.pose.position.angleTo(goal).toDegrees()
                GlobalStore.autonPose = drive.pose
            }
        })

        scheduler.schedule(Sequential("Close Auton Safe") {
            Command("Start") {
                WaitFor { isStarted }
                flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
            }

            // preload
            +autonManager.runPath(0)
            +fire

            // row 2
            +autonManager.runPathChain(
                listOf(1, 2)
            )
            +Parallel {
                +autonManager.runPath(3)
                Command {
                    drive.follower.setMaxPower(1.0)
                    ParametricCallback(drive.follower, 0, row2OuttakeT) {
                        intake.power = -1.0
                    }
                }
            }
            +fire

            // ramp
            repeat(2) {
                +gateCycle
                +fire
            }

            // row 3
            +autonManager.runPathChain(listOf(7, 8))
            +Parallel {
                +autonManager.runPath(9) {
                    drive.follower.setMaxPower(1.0)
                }
                Command {
                    ParametricCallback(drive.follower, 0, row3OuttakeT) {
                        intake.power = -1.0
                    }
                }
            }
            +fire

            // row 1
            +autonManager.runPathChain(listOf(10, 11))
            +fire

            // ending
            +endAuton
        })
    }

    companion object {
        @JvmField
        var upDuration = 200L

        @JvmField
        var downDuration = 50L

        @JvmField
        var gatePower = 0.6

        @JvmField
        var gateCounterRotate = -0.175

        @JvmField
        var gateDuration = 500L

        @JvmField
        var rampDuration = 1500L

        @JvmField
        var rampOutakeT = 0.05

        @JvmField
        var row2OuttakeT = 0.75

        @JvmField
        var row3OuttakeT = 0.375
    }
}

suspend fun Command.ParametricCallback(
    follower: Follower,
    pathIndex: Int,
    tValue: Double,
    callback: suspend Command.() -> Unit
) {
    sync()
    WaitFor { follower.currentPathNumber > pathIndex || (follower.chainIndex == pathIndex && follower.currentTValue >= tValue) }
    callback()
}