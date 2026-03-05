package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
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
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = TeleOpDrive(this, isRed)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })

        turret.targetingMode = Turret.TargetingMode.RELATIVE
        turret.target = 180.0

        var turretOff = false

        val autoAdjust = AutoAdjust(this, flyWheel, hood, { drive.pose }, isRed)

        drive.pose = GlobalStore.autonPose ?: Pose2d(112.0, 137.0, -90.0).mirror(!isRed)
        GlobalStore.autonPose = null

        val rapidFire = Command("Rapid Fire", {
            sorter.frontPod.isUp = false
            sorter.sidePod.isUp = false
            sorter.backPod.isUp = false

            flyWheel.state = FlyWheel.FlyWheelState.IDLE
            turret.targetingMode = Turret.TargetingMode.RELATIVE
            turretTarget = turret.angle
        }) {
            flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
            turret.targetingMode = if (turretOff) {
                Turret.TargetingMode.RELATIVE
            } else {
                Turret.TargetingMode.GLOBAL
            }

            WaitFor { turret.atTarget && !turret.inDeadzone && turret.isSteady }

            val minRPM = { autoAdjust.minRapidRPM - FlyWheel.Controller.rpmThreshold }
            val maxRPM = { flyWheel.shootingRPM + FlyWheel.Controller.rpmThreshold }

            val atRPM = { flyWheel.rpm in minRPM()..maxRPM() }
            WaitFor { atRPM() }

            sorter.frontPod.isUp = true
            SleepFor { upDuration }
            sorter.frontPod.isUp = false
            SleepFor { downDuration }

            sorter.sidePod.isUp = true
            SleepFor { upDuration }
            sorter.sidePod.isUp = false
            SleepFor { downDuration }

            sorter.backPod.isUp = true
            SleepFor { upDuration }
            sorter.backPod.isUp = false
            SleepFor { downDuration }

            flyWheel.state = FlyWheel.FlyWheelState.IDLE
            turret.targetingMode = Turret.TargetingMode.RELATIVE
            turretTarget = turret.angle
        }

        scheduler.schedule(Command("Rapid Fire Scheduler") {
            OpModeLoop(this@Teleop) {
                if (!gp1.prev.rightBumper && gp1.current.rightBumper) {
                    rapidFire.cancel()
                    scheduler.schedule(rapidFire)
                }
            }
        })

        scheduler.schedule(Command("Power adjusts") {
            OpModeLoop(this@Teleop) {
                if (gp1.current.dPad.down && !gp1.prev.dPad.down) {
                    drive.pose = Pose2d(112.0, 137.0, -90.0).mirror(!isRed)
                } else if (gp1.current.dPad.left && !gp1.prev.dPad.left) {
                    drive.pose = Pose2d(137.0, 8.5, 180.0).mirror(!isRed)
                }

                if (gp1.current.leftBumper && !gp1.prev.leftBumper) {
                    turretOff = !turretOff
                }

//                hood.target = hoodOverride
                if (intakeOverride == Double.NEGATIVE_INFINITY) {
                    val intakePower = gp1.current.rightTrigger - gp1.current.leftTrigger
                    intake.power = intakePower

                    if (intakePower < -0.5) drive.useGateAssist = false
                } else {
                    intake.power = intakeOverride
                }

                val pose = drive.pose
                tel.addData("pose", pose)

                if (turret.targetingMode == Turret.TargetingMode.GLOBAL) {
                    turret.target = normalizeDegrees(
                        pose.angleTo(
                            Vec2d(144.0 - 4.0, 144.0 - 4.0).mirror(!isRed)
                        ).toDegrees()
                    )
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
        var intakeOverride = Double.NEGATIVE_INFINITY

        @JvmField
        var turretTarget = 180.0
    }
}