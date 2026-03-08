package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.toRadians
import com.pedropathing.geometry.BezierLine
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.absoluteValue

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
        val autoAdjust = AutoAdjust(
            this, flyWheel, hood, { drive.pose }, { drive.velocity.position }, isRed
        )

        turret.targetingMode = if (liveTracking) Turret.TargetingMode.GLOBAL else Turret.TargetingMode.RELATIVE
        turret.target = 180.0

        var manualMode = false
        var lastIntake = 0L

        val rapidFire = Command("Rapid Fire", {
            sorter.resetKickers()
        }) {
            flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
            turret.targetingMode = Turret.TargetingMode.GLOBAL

            WaitFor {
                turret.atTarget && !turret.inDeadzone && turret.isSteady && drive.inZonePartial
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

        val autoPark = Command("Auto Park", {
            drive.isTeleopDrive = true
        })
        {
            drive.isTeleopDrive = false
            drive.follower.followPath(
                drive.follower.pathBuilder()
                    .addPath(
                        Path(
                            BezierLine(
                                { drive.pose.toPedro() },
                                Pose2d(105.0, 33.0, 0.0).mirror(!isRed).toPedro()
                            )
                        )
                    )
                    .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                            { drive.pose.radians },
                            (0.0).toRadians(),
                            0.5
                        )
                    )
                    .build()
            )
            WaitFor { isStopRequested || drive.follower.atParametricEnd() }
            drive.follower.breakFollowing()
            drive.isTeleopDrive = true
        }

        scheduler.schedule(Command {
            OpModeLoop(this@Teleop) {
                // manual
                val intakePower = gp1.current.rightTrigger - gp1.current.leftTrigger

                intake.power = intakePower

                if (intakePower > intakeThreshold) lastIntake = System.currentTimeMillis()

                if (intakePower.absoluteValue < intakeThreshold &&
                    System.currentTimeMillis() < lastIntake + outtakeBurstDuration
                ) {
                    intake.power = outtakeBurstPower
                    drive.useGateAssist = false
                }

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
                if (!gp1.prev.y && gp1.current.y) {
                    if (!drive.follower.teleopDrive) {
                        autoPark.cancel()
                    } else {
                        scheduler.schedule(autoPark)
                    }
                }

                val manualAim = gp2.current.rightJoyStick.vector
                if (manualAim.magnitude() > 0.5) {
                    manualMode = true
                    autoAdjust.enabled = false
                }

                if (!gp2.prev.dPad.down && gp2.current.dPad.down) {
                    println("disable manual")
                    manualMode = false
                    autoAdjust.enabled = true
                    turret.targetingMode =
                        if (liveTracking) Turret.TargetingMode.GLOBAL else Turret.TargetingMode.RELATIVE
                }

                // auto
                if (intakePower < -0.5) drive.useGateAssist = false

                if (!manualMode) {
                    if (turret.targetingMode == Turret.TargetingMode.GLOBAL) {
                        turret.target = autoAdjust.turretAngle
                    } else {
                        turret.target = 180.0
                    }
                } else {
                    if (manualAim.magnitude() > 0.5) {
                        turret.target = -manualAim.angle().toDegrees()
                        turret.targetingMode = Turret.TargetingMode.GLOBAL
                    } else {
                        turret.target = 180.0
                        turret.targetingMode = Turret.TargetingMode.RELATIVE
                    }
                    val shooterTarget = AutoAdjust.getTarget(72.0)
                    flyWheel.shootingRPM = shooterTarget.first
                    hood.target = shooterTarget.second
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

        @JvmField
        var intakeThreshold = 0.2

        @JvmField
        var outtakeBurstDuration = 500L

        @JvmField
        var outtakeBurstPower = -1.0
    }
}