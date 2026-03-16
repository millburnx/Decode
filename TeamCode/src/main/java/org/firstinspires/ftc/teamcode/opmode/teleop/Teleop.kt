package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.commands.AutoPark
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.IndicatorLight
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
        GlobalStore.useKF = true

        val indicatorLight = IndicatorLight(this)

        val limelight = Limelight(this)
        val drive = TeleOpDrive(this, isRed, limelight)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        limelight.turretHeading = { turret.angle }
        limelight.drawPose = { pose ->
            drive.drawRobot(pose, "llPose", "yellow")
        }
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val sorter = Sorter(this, indicatorLight)
        val autoAdjust = AutoAdjust(
            this, flyWheel, hood, { drive.pose }, { drive.velocity.position }, { drive.inZoneFarPartial }, isRed
        )

        limelight.onSwitch = {
            turret.targetingMode = Turret.TargetingMode.GLOBAL
        }

        drive.pose = GlobalStore.autonPose ?: Pose2d(112.0, 137.0, -90.0).mirror(!isRed)
        turret.targetingMode =
            if (liveTracking && GlobalStore.autonPose != null) Turret.TargetingMode.GLOBAL else Turret.TargetingMode.RELATIVE
        GlobalStore.autonPose = null
        turret.target = 180.0

        var manualMode = false
        var lastIntake = 0L

        var greenSlot: Sorter.Pods? = null
        var greenPosition = 0

        val rapidFire = Command("Rapid Fire", {
            sorter.resetKickers()
        }) {
            flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
            turret.targetingMode = Turret.TargetingMode.GLOBAL

            val turretReady = turretReady@{
                val deadzone = turret.inDeadzone
                indicatorLight.stateFlags[IndicatorLight.State.TURRET_DEADZONE] = deadzone
                return@turretReady turret.atTarget && !deadzone && turret.isSteady
            }
            val zoneCheck = zoneCheck@{
                val zoneCheckResult = if (useZoneCheck) drive.inZonePartial else true
                indicatorLight.stateFlags[IndicatorLight.State.OUT_OF_ZONE] = !zoneCheckResult
                return@zoneCheck zoneCheckResult
            }

            WaitFor { zoneCheck() && turretReady() }
            GlobalStore.useKF = false
            indicatorLight.stateFlags[IndicatorLight.State.OUT_OF_ZONE] = false
            indicatorLight.stateFlags[IndicatorLight.State.TURRET_DEADZONE] = false
            WaitFor { flyWheel.atRPM }

            indicatorLight.stateFlags[IndicatorLight.State.FIRING] = true

            val greenSlotCopy = greenSlot
            if (greenSlotCopy != null) {
                val firingOrder = sorter.getFiringOrder(greenSlotCopy, greenPosition)
                sorter.run { rapidFire(firingOrder, sortingUpDuration to sortingDownDuration) }
                greenSlot = null // reset sorting
                greenPosition = 0
            } else {
                sorter.run {
                    rapidFire(
                        firingSpeed = getFiringSpeed(drive.inZoneFarPartial)
                    )
                }
            }

            indicatorLight.stateFlags[IndicatorLight.State.FIRING] = false
            flyWheel.state = FlyWheel.FlyWheelState.IDLE
            if (!liveTracking) turret.targetingMode = Turret.TargetingMode.RELATIVE
            GlobalStore.useKF = true
        }

        val autoPark = AutoPark(drive, isRed) { !isStopRequested }

        scheduler.schedule(Command {
            OpModeLoop(this@Teleop) {
                // manual
                val intakePower = gp1.current.rightTrigger - gp1.current.leftTrigger

                intake.targetPower = intakePower

                if (intakePower > intakeThreshold) lastIntake = System.currentTimeMillis()

                if (intakePower.absoluteValue < intakeThreshold &&
                    System.currentTimeMillis() < lastIntake + outtakeBurstDuration
                ) {
                    intake.targetPower = outtakeBurstPower
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

                // manual sorting
                if (!gp2.prev.dPad.up && gp2.current.dPad.up) {
                    greenSlot = Sorter.Pods.FRONT
                }
                if (!gp2.prev.dPad.left && gp2.current.dPad.left) {
                    greenSlot = Sorter.Pods.SIDE
                }
                if (!gp2.prev.dPad.down && gp2.current.dPad.down) {
                    greenSlot = Sorter.Pods.BACK
                }

                if (!gp2.prev.x && gp2.current.x) {
                    greenPosition = 0
                    println("green pos $greenPosition")
                }
                if (!gp2.prev.y && gp2.current.y) {
                    greenPosition = 1
                    println("green pos $greenPosition")
                }
                if (!gp2.prev.b && gp2.current.b) {
                    greenPosition = 2
                    println("green pos $greenPosition")
                }

                indicatorLight.stateFlags[IndicatorLight.State.SORTING] = greenSlot != null

                tel.addData("green pos", greenPosition)
                tel.addData("green slot", greenSlot?.ordinal ?: -1.0)

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
        var upDuration = 125L

        @JvmField
        var downDuration = 50L

        @JvmField
        var farUpDuration = 175L

        @JvmField
        var farDownDuration = 50L

        @JvmField
        var sortingUpDuration = 500L

        @JvmField
        var sortingDownDuration = 100L

        @JvmField
        var liveTracking = true

        @JvmField
        var intakeThreshold = 0.2

        @JvmField
        var outtakeBurstDuration = 750L

        @JvmField
        var outtakeBurstPower = -1.0

        @JvmField
        var useZoneCheck = true

        fun getFiringSpeed(isFarZone: Boolean = false): Pair<Long, Long> {
            if (isFarZone) return farUpDuration to farDownDuration
            return upDuration to downDuration
        }
    }
}