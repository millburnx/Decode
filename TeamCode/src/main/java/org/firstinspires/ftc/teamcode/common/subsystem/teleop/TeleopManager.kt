package org.firstinspires.ftc.teamcode.common.subsystem.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.commands.AutoPark
import org.firstinspires.ftc.teamcode.common.commands.RapidFire
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class TeleopManager(
    val opMode: OpMode,
    val drive: TeleOpDrive,
    val turret: Turret,
    val flyWheel: FlyWheel,
    val hood: Hood,
    val sorter: Sorter,
    val intake: Intake,
    val autoAdjust: AutoAdjust,
    val indicatorLight: IndicatorLight,
    var isRed: Boolean, // var in case we select the wrong side
) : Subsystem("Teleop Manager") {
    val intakeManager = TeleopIntakeManager(opMode, intake, sorter)
    val turretManager = TeleopTurretManager(opMode, turret, autoAdjust, { isRed }) { !autoAdjust.enabled }
    val sorterManager = TeleopSorterManager(opMode, drive, sorter, intake, indicatorLight)

    val autoPark = AutoPark(drive, isRed) { !opMode.isStopRequested }
    val rapidFire =
        RapidFire(
            drive,
            sorterManager,
            turret,
            flyWheel,
            indicatorLight,
            autoAdjust,
            { liveTracking },
            { isRed },
        )

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            with(opMode) {
                gp1.y.ifPressed {
                    if (!drive.follower.teleopDrive) {
                        autoPark.cancel()
                        return@ifPressed
                    }
                    scheduler.schedule(autoPark)
                }

                tel.addData("rotationVelocity", drive.velocity.heading)

                gp1.rightBumper.ifPressed {
                    println("triggering rapidfire")
                    rapidFire.trigger(scheduler)
                }

                // manual controls
                if (gp2.current.dPad.right) autoAdjust.enabled = true

                val manualJoystick = gp2.current.rightJoyStick
                if (manualJoystick.magnitude > 0.5) autoAdjust.enabled = false

                if (!autoAdjust.enabled) {
                    val target = DATA.ceilingEntry(72.0)?.value ?: (3100.0 to 0.0)
                    flyWheel.shootingRPM = target.first
                    hood.target = target.second
                }
            }
        }
    }

    companion object {
        @JvmField
        var rotationVelocityThreshold = 0.5

        @JvmField
        var sortingUpDuration = 500L

        @JvmField
        var sortingDownDuration = 100L

        @JvmField
        var liveTracking: Boolean = false
    }
}
