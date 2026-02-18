package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.util.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.hardware.fromPedro
import org.firstinspires.ftc.teamcode.common.subsystem.*
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

        val rapidFire = Command("Rapid Fire") {
            FlyWheel.override = true
            FlyWheel.overridePower = rapidPower
            SleepFor { spinUp }

            sorter.isFrontUp = true
            SleepFor { upDuration }
            sorter.isFrontUp = false
            SleepFor { downDuration }

            sorter.isSideUp = true
            SleepFor { upDuration }
            sorter.isSideUp = false
            SleepFor { downDuration }

            sorter.isBackUp = true
            SleepFor { upDuration }
            sorter.isBackUp = false

            FlyWheel.overridePower = 0.0
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
            OpModeLoop(this@Teleop) {
                pedro.update()
                pedro.setTeleOpDrive(
                    gp1.current.leftJoyStick.y,
                    gp1.current.leftJoyStick.x,
                    gp1.current.rightJoyStick.x,
                    !fieldCentric
                )

                hood.target = hoodOverride
                if (intakeOverride == Double.NEGATIVE_INFINITY) {
                    intake.power = gp1.current.rightTrigger - gp1.current.leftTrigger
                } else {
                    intake.power = intakeOverride
                }

                turret.target = turretTarget
                turret.targetingMode =
                    if (turretGlobal) Turret.TargetingMode.GLOBAL else Turret.TargetingMode.RELATIVE
            }
        })
    }

    companion object {
        @JvmField
        var upDuration = 100L

        @JvmField
        var downDuration = 50L

        @JvmField
        var hoodOverride = 0.5

        @JvmField
        var intakeOverride = Double.NEGATIVE_INFINITY

        @JvmField
        var fieldCentric = false

        @JvmField
        var turretTarget = 180.0

        @JvmField
        var turretGlobal = false

        @JvmField
        var rapidPower = 1.0

        @JvmField
        var spinUp = 1000L
    }
}