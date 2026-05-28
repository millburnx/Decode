package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleOpDrive
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleopManager
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleopManager.Companion.liveTracking
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@TeleOp
class TeleopRed : Teleop(true)

@TeleOp
class TeleopBlue : Teleop(false)

@Configurable
open class Teleop(
    val isRed: Boolean,
) : OpMode() {
    override fun run() {
        GlobalStore.useKF = true

        val indicatorLight = IndicatorLight(this)
        val limelight = Limelight(this)
        val intake = Intake(this)
        val drive = TeleOpDrive(this, isRed, limelight, intake)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        val sorter = Sorter(this, indicatorLight)
        limelight.turretHeading = { turret.angle }
        limelight.drawPose = { drive.drawRobot(it, "llPose", "yellow") }
        limelight.onSwitch =
            {
                indicatorLight.fullFlash()
                if (liveTracking) turret.setGlobal() else turret.setRelative()
            }
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val autoAdjust = AutoAdjust(this, flyWheel, hood, drive, isRed) { drive.inZoneFarPartial }

        drive.pose = GlobalStore.autonPose ?: Pose2d(112.0, 137.0, -90.0).mirror(!isRed)
        turret.targetingMode =
            if (liveTracking && GlobalStore.autonPose != null) Turret.TargetingMode.GLOBAL else Turret.TargetingMode.RELATIVE
        GlobalStore.autonPose = null
        turret.target = 180.0
        intake.power = 0.0

        val teleopManager =
            TeleopManager(
                this,
                drive,
                turret,
                flyWheel,
                hood,
                sorter,
                intake,
                autoAdjust,
                indicatorLight,
                isRed,
            )

        scheduler.schedule(
            Command {
                OpModeLoop(this@Teleop) {
                }
            },
        )
    }

    companion object {
        @JvmField
        var upDuration = 125L

        @JvmField
        var downDuration = 50L

        @JvmField
        var farUpDuration = 175L

        @JvmField
        var farDownDuration = 50L

        @JvmField
        var outtakeBurstDuration = 750L

        @JvmField
        var outtakeBurstPower = -1.0

        @JvmField
        var useZoneCheck = true

        @JvmField
        var minDistToGoal = 38.0
    }
}
