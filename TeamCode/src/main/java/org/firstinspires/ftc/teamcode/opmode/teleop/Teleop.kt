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
import org.firstinspires.ftc.teamcode.common.hardware.fromPedro
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants

@Configurable
@TeleOp
class Teleop : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val pedro = Constants.createManualFusionFollower(hardwareMap, { deltaTime })
        val turret = Turret(
            this,
            { Pose2d.fromPedro(pedro.pose).heading },
            { pedro.angularVelocity },
            { voltageSensor.voltage }
        )

        val autoAdjust = AutoAdjust(this, flyWheel, hood, { Pose2d.fromPedro(pedro.pose) })

        scheduler.schedule(Command {
            SleepFor { 1000 }
            pedro.pose = Pose2d(72.0, 72.0, 0.0).toPedro()
        })

        val rapidFire = Command("Rapid Fire") {
//            FlyWheel.override = true
//            FlyWheel.overridePower = rapidPower
//            SleepFor { spinUp }
            flyWheel.state = FlyWheel.FlyWheelState.SHOOTING

            val enoughRPM = { flyWheel.rpm > autoAdjust.minRapidRPM - FlyWheel.Controller.rpmThreshold}
            SleepFor(spinUp) {
                flyWheel.rpm > flyWheel.targetRpm - maxLowerRPM
            }

            WaitFor { enoughRPM() }
            sorter.isFrontUp = true
            SleepFor { upDuration }
            sorter.isFrontUp = false
            SleepFor { downDuration }

            WaitFor { enoughRPM() }
            sorter.isSideUp = true
            SleepFor { upDuration }
            sorter.isSideUp = false
            SleepFor { downDuration }

            WaitFor { enoughRPM() }
            sorter.isBackUp = true
            SleepFor { upDuration }
            sorter.isBackUp = false

//            FlyWheel.overridePower = 0.0
            flyWheel.state = FlyWheel.FlyWheelState.IDLE
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
            pedro.update()
            pedro.startTeleopDrive(false)

            val canvas = PanelsField.field
            canvas.setOffsets(PanelsField.presets.PEDRO_PATHING)

            OpModeLoop(this@Teleop) {
                pedro.update()
                pedro.setTeleOpDrive(
                    gp1.current.leftJoyStick.y,
                    gp1.current.leftJoyStick.x,
                    gp1.current.rightJoyStick.x,
                    !fieldCentric
                )

//                hood.target = hoodOverride
                if (intakeOverride == Double.NEGATIVE_INFINITY) {
                    intake.power = gp1.current.rightTrigger - gp1.current.leftTrigger
                } else {
                    intake.power = intakeOverride
                }

                val pose = Pose2d.fromPedro(pedro.pose)
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

//        @JvmField
//        var hoodOverride = 0.5

        @JvmField
        var intakeOverride = Double.NEGATIVE_INFINITY

        @JvmField
        var fieldCentric = false

        @JvmField
        var turretTarget = 180.0

        @JvmField
        var turretGlobal = false

        //        @JvmField
//        var rapidPower = 1.0
//
        @JvmField
        var spinUp = 2000L

        @JvmField
        var autoAim = true

        @JvmField
        var maxLowerRPM = 200
    }
}