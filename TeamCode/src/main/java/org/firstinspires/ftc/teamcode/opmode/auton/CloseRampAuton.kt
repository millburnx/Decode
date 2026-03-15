package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Disabled
@Autonomous
class CloseRampRed : CloseRampAuton(true)

@Disabled
@Autonomous
class CloseRampBlue : CloseRampAuton(false)

@Configurable
open class CloseRampAuton(var isRed: Boolean) : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = Drive(this)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        val autoAdjust = AutoAdjust(this, flyWheel, hood, { drive.pose }, { Vec2d() }, isRed)

        val autonManager = AutonManager(this, drive, "Close-RampIntake", isMirrored = !isRed)

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
                drive.follower.setTeleOpDrive(gatePower, 0.0, 0.0, false)

                SleepFor { gateDuration }

                drive.follower.setTeleOpDrive(0.0, 0.0, 0.0)

                SleepFor { rampDuration }
            }
        }

        val gateCycle = Sequential {
            +autonManager.runPath(4)
            +gateIntake
            +autonManager.runPath(6) {
                it.addParametricCallback(rampOutakeT) {
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

        val goal = if (autonManager.isMirrored) Vec2d(0.0, 144.0) else Vec2d(144.0, 144.0)

        scheduler.schedule(Command("general") {
            OpModeLoop(this@CloseRampAuton) {
                turret.targetingMode = Turret.TargetingMode.GLOBAL
                turret.target = drive.pose.position.angleTo(goal).toDegrees()
            }
        })

        scheduler.schedule(Sequential("Close Auton Safe") {
            Command("Start") {
                WaitFor { isStarted }
                flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
            }
            +autonManager.runPath(0)
            Command { SleepFor { stablizationTime } }
            +fire
            +autonManager.runPath(1)
            +autonManager.runPath(2)
            +autonManager.runPath(3)
            Command { SleepFor { stablizationTime } }
            +fire
            +gateCycle
            +fire
            +gateCycle
            +fire
            +gateCycle
            +fire
            +gateCycle
            +fire
            +endAuton
        })
    }

    companion object {
        @JvmField
        var upDuration = 150L

        @JvmField
        var downDuration = 50L

        @JvmField
        var stablizationTime = 200L

        @JvmField
        var gatePower = 0.6

        @JvmField
        var gateDuration = 500L
        @JvmField
        var rampDuration = 1500L

        @JvmField
        var rampOutakeT = 0.25
    }
}