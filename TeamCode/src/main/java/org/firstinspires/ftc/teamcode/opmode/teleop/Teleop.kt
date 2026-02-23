package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.PanelsField
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class Teleop : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = TeleOpDrive(this)
        val turret = Turret(
            this,
            { drive.pose.heading },
            { drive.velocity.heading },
            { voltageSensor.voltage }
        )

        val autoAdjust = AutoAdjust(this, flyWheel, hood, { drive.pose })

        scheduler.schedule(Command {
            SleepFor { 1000 }
            drive.pose = Pose2d(72.0, 72.0, 0.0)
        })

        val rapidFire = Command("Rapid Fire", {
            sorter.frontPod.isUp = false
            sorter.sidePod.isUp = false
            sorter.backPod.isUp = false

            flyWheel.state = FlyWheel.FlyWheelState.IDLE
            autoAim = false
            turretTarget = turret.angle
        }) {
            WaitFor { turret.atTarget && !turret.inDeadzone }

            flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
            autoAim = true

            val enoughRPM = { flyWheel.rpm > autoAdjust.minRapidRPM - FlyWheel.Controller.rpmThreshold }
            SleepFor(spinUp) {
                flyWheel.rpm > flyWheel.targetRpm - maxLowerRPM
            }

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
            autoAim = false
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
            val canvas = PanelsField.field
            canvas.setOffsets(PanelsField.presets.PEDRO_PATHING)

            OpModeLoop(this@Teleop) {
//                hood.target = hoodOverride
                if (intakeOverride == Double.NEGATIVE_INFINITY) {
                    intake.power = gp1.current.rightTrigger - gp1.current.leftTrigger
                } else {
                    intake.power = intakeOverride
                }

                val pose = drive.pose
                canvas.moveCursor(pose.x, pose.y)
                canvas.setStyle(fill = "none", outline = "white", width = 1.5)
                canvas.circle(8.0)
                val lookPos = pose + Vec2d(1.0, 0.0).rotate(pose.radians) * 8.0
                canvas.line(lookPos.x, lookPos.y)

                tel.addData("pose", pose)

                if (autoAim) {
                    turret.targetingMode = Turret.TargetingMode.GLOBAL
                    turret.target = normalizeDegrees(
                        pose.angleTo(
                            Vec2d(144.0 - 4.0, 144.0 - 4.0)
                        ).toDegrees()
                    )
                } else {
                    turret.target = turretTarget
                    turret.targetingMode =
                        if (turretGlobal) Turret.TargetingMode.GLOBAL else Turret.TargetingMode.RELATIVE
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
        var intakeOverride = Double.NEGATIVE_INFINITY

        @JvmField
        var turretTarget = 180.0

        @JvmField
        var turretGlobal = false

        @JvmField
        var spinUp = 2000L

        @JvmField
        var autoAim = false

        @JvmField
        var maxLowerRPM = 200
    }
}