package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@TeleOp
class TeleopRed : Teleop(true)

@TeleOp
class TeleopBlue : Teleop(false)

@Configurable
open class Teleop(val isRed: Boolean) : OpMode() {
    override fun run() {
        val limelight = Limelight(this)
        val drive = TeleOpDrive(this, true, limelight)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        limelight.turretHeading = { turret.angle }
        limelight.drawPose = { pose ->
            drive.drawRobot(pose, "llPose", "yellow")
        }
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val sorter = Sorter(this)

        turret.targetingMode = if (liveTracking) Turret.TargetingMode.GLOBAL else Turret.TargetingMode.RELATIVE
        turret.target = 180.0

        val autoAdjust = AutoAdjust(
            this,
            flyWheel,
            hood,
            { drive.pose },
            { drive.velocity.position },
            isRed
        )

        val rapidFire = Command("Rapid Fire", {
            sorter.resetKickers()
        }) {
            flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
            turret.targetingMode = Turret.TargetingMode.GLOBAL

            WaitFor {
                turret.atTarget && !turret.inDeadzone && turret.isSteady
            }

            val minRPM = { flyWheel.shootingRPM - FlyWheel.Controller.rpmThreshold }
            val maxRPM = { flyWheel.shootingRPM + FlyWheel.Controller.rpmThreshold }

            val atRPM = { flyWheel.rpm in minRPM()..maxRPM() }

            WaitFor { atRPM() }

            sorter.frontPod.kick(this, upDuration, downDuration)
            sorter.sidePod.kick(this, upDuration, downDuration)
            sorter.backPod.kick(this, upAxonDuration, downAxonDuration)

            flyWheel.state = FlyWheel.FlyWheelState.IDLE
            if (!liveTracking) turret.targetingMode = Turret.TargetingMode.RELATIVE
        }

        scheduler.schedule(Command {
            OpModeLoop(this@Teleop) {
                // manual
                val intakePower = gp1.current.rightTrigger - gp1.current.leftTrigger
                intake.power = intakePower

                if (!gp1.prev.dPad.left && gp1.current.dPad.left) {
                    drive.pose = Pose2d(112.0, 137.0, -90.0).mirror(!isRed)
                }
                if (!gp1.prev.dPad.up && gp1.current.dPad.up) {
                    drive.pose = Pose2d(137.0, 8.5, 180.0).mirror(!isRed)
                }
                if (!gp1.prev.leftBumper && gp1.current.leftBumper) {
                    limelight.localizationState = Limelight.LocalizationState.NONE
                }
                if (!gp1.prev.rightBumper && gp1.current.rightBumper) {
                    rapidFire.cancel()
                    scheduler.schedule(rapidFire)
                }

                // auto
                if (intakePower < -0.5) drive.useGateAssist = false

                if (turret.targetingMode == Turret.TargetingMode.GLOBAL) {
                    turret.target = autoAdjust.turretAngle
                } else {
                    turret.target = 180.0
                }
            }
        })
    }

    companion object {
        @JvmField
        var upDuration = 150L

        @JvmField
        var downDuration = 50L

        @JvmField
        var upAxonDuration = 400L

        @JvmField
        var downAxonDuration = 50L

        @JvmField
        var liveTracking = true
    }
}