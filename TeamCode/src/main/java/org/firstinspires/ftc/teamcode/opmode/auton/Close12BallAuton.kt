package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Autonomous(preselectTeleOp = "TeleopRed")
class Close12BallRed : Close12BallAuton(true)

@Autonomous(preselectTeleOp = "TeleopBlue")
class Close12BallBlue : Close12BallAuton(false)

@Configurable
open class Close12BallAuton(var isRed: Boolean) : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = Drive(this)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        val autoAdjust = AutoAdjust(this, flyWheel, hood, drive, isRed)

        val autonManager = AutonManager(this, drive, "Close-12Ball-6Ramp-AllRows", isMirrored = !isRed)

        val fire = Command {
            intake.power = -1.0
            WaitFor { turret.atTarget && turret.isSteady }
            val atRPM = { flyWheel.atRPM }
            WaitFor { atRPM() }
            sorter.run { rapidFire() }
            intake.power = 1.0
        }

        val goal = if (autonManager.isMirrored) Vec2d(0.0, 144.0) else Vec2d(144.0, 144.0)

        scheduler.schedule(Command("general") {
            OpModeLoop(this@Close12BallAuton) {
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
            +autonManager.runPath(0)
            Command { SleepFor { stablizationTime } }
            +fire
            +autonManager.runPath(1)
            +autonManager.runPath(2)
            Command { SleepFor { stablizationTime } }
            +fire
            +autonManager.runPath(3)
            +autonManager.runPath(4)
            Command { intake.power = 0.0 }
            +autonManager.runPath(5)
            Command {
                drive.follower.breakFollowing()
                drive.follower.startTeleopDrive(true)
                drive.follower.setTeleOpDrive(gatePower, 0.0, 0.0)
                SleepFor { gateDuration }
            }
            +autonManager.runPath(6)
            Command { SleepFor { stablizationTime } }
            +fire
            +autonManager.runPath(7)
            +autonManager.runPath(8)
            +autonManager.runPath(9)
            Command { SleepFor { stablizationTime } }
            +fire
            Command { intake.power = 0.0; FlyWheel.override = true; FlyWheel.overridePower = 0.0 }
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
        var gatePower = 0.5

        @JvmField
        var gateDuration = 1000L
    }
}