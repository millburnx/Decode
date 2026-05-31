package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Autonomous(preselectTeleOp = "TeleopRed")
class FarHomoRed : FarHomoAuton(true)

@Autonomous(preselectTeleOp = "TeleopBlue")
class FarHomoBlue : FarHomoAuton(false)

@Configurable
open class FarHomoAuton(
    var isRed: Boolean,
) : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = Drive(this)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        val autoAdjust = AutoAdjust(this, flyWheel, hood, drive, isRed) { true }

        val autonManager = AutonManager(this, drive, "Far-homo", isMirrored = !isRed)

        val fire =
            Command {
                intake.power = -1.0
                WaitFor { turret.atTarget && turret.isSteady }
                val atRPM = { flyWheel.atRPM }
                WaitFor { atRPM() }
                sorter.run { rapidFire(firingSpeed = upDuration to downDuration) }
                intake.power = 1.0
            }

        val endAuton =
            Command {
                intake.power = 0.0
                FlyWheel.override = true
                FlyWheel.overridePower = 0.0
                println("auton end ${matchTimer.seconds()}")
            }

        scheduler.schedule(
            Command("general") {
                OpModeLoop(this@FarHomoAuton) {
                    turret.targetingMode = Turret.TargetingMode.GLOBAL
//                turret.target = drive.pose.position.angleTo(goal).toDegrees()
                    turret.target = autoAdjust.turretAngle
                    GlobalStore.autonPose = drive.pose
                }
            },
        )

        scheduler.schedule(
            Sequential("Far Auton") {
                Command("Start") {
                    WaitFor { isStarted }
                    flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
                    autoAdjust.rpmOffset = rpmOffset
                }
                +autonManager.runPath(0)
                +fire

                +autonManager.runPath(1)
                +autonManager.runPath(2)
                Command { SleepFor { stablizationTime } }
                +fire

                repeat(4) {
                    +autonManager.runPath(3)
                    +autonManager.runPath(4)
                    Command {
                        SleepFor {
                            intake.power = -1.0
                            stablizationTimeLong
                        }
                    }
                    +fire
                }

                +autonManager.runPath(5)
                +endAuton
            },
        )
    }

    companion object {
        @JvmField
        var upDuration = 125L

        @JvmField
        var downDuration = 50L

        @JvmField
        var stablizationTime = 200L

        @JvmField
        var stablizationTimeLong = 1000L

        @JvmField
        var rpmOffset = -200.0
    }
}
