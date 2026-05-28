package org.firstinspires.ftc.teamcode.common.commands

import com.millburnx.cmdx.Command
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.millburnx.cmdxpedro.util.mirror
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.subsystem.AutoAdjust
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.IndicatorLight
import org.firstinspires.ftc.teamcode.common.subsystem.Turret
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleOpDrive
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleopManager.Companion.sortingDownDuration
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleopManager.Companion.sortingUpDuration
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleopSorterManager
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop.Companion.downDuration
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop.Companion.farDownDuration
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop.Companion.farUpDuration
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop.Companion.minDistToGoal
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop.Companion.upDuration
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop.Companion.useZoneCheck

class RapidFire(
    val drive: TeleOpDrive,
    val sorterManager: TeleopSorterManager,
    val turret: Turret,
    val flyWheel: FlyWheel,
    val indicatorLight: IndicatorLight,
    val autoAdjust: AutoAdjust,
    val liveTracking: () -> Boolean,
    val isRed: () -> Boolean,
) {
    val sorter = sorterManager.sorter

    fun trigger(scheduler: CommandScheduler) {
        scheduler.schedule(
            Command("Rapid Fire Intermediate") {
                command.cancel()
                sync()
                scheduler.schedule(command)
            },
        )
    }

    private val command =
        Command("Rapid Fire", {
            if (liveTracking() && autoAdjust.enabled) turret.setGlobal() else turret.setRelative()
            sorterManager.sortingConfig.pod = null
            sorter.resetKickers()
            flyWheel.setIdle()
            GlobalStore.useKF = true
        }) {
            if (autoAdjust.enabled) turret.setGlobal()
            flyWheel.setShooting()

            while (!zoneCheck() || !turret.isReady || !flyWheel.atRPM) {
                println("readyness ${zoneCheck()} ${turret.isReady} ${flyWheel.atRPM}")
                indicatorLight.stateFlags[IndicatorLight.State.TURRET_DEADZONE] = turret.inDeadzone
                sync()
            }

            indicatorLight.stateFlags[IndicatorLight.State.OUT_OF_ZONE] = false
            indicatorLight.stateFlags[IndicatorLight.State.TURRET_DEADZONE] = false
            indicatorLight.stateFlags[IndicatorLight.State.FIRING] = true

            GlobalStore.useKF = false

            val sortingPod = sorterManager.greenPod
            val sortingPattern = sorterManager.sortingConfig.pattern
            if (sortingPod != null && sortingPattern != null) { // same as sorter.isSorting but prevents race conditions during usage
                val firingOrder = sorter.getFiringOrder(sortingPod, sortingPattern)

                sorter.run {
                    rapidFire(firingOrder, sortingUpDuration to sortingDownDuration)
                }
                sorterManager.sortingConfig.pod = null
            } else {
                sorter.run {
                    rapidFire(firingSpeed = getFiringSpeed(drive.inZoneFarPartial))
                }
            }

            indicatorLight.stateFlags[IndicatorLight.State.FIRING] = false
            GlobalStore.useKF = true
            flyWheel.setIdle()
            if (!liveTracking() || !autoAdjust.enabled) turret.setRelative()
        }

    private fun zoneCheck(): Boolean {
        if (!useZoneCheck) return true
        val goal = AutoAdjust.goal.mirror(isRed())
        val finalResult = drive.inZonePartial && (drive.pose.distanceTo(goal) > minDistToGoal)
        indicatorLight.stateFlags[IndicatorLight.State.OUT_OF_ZONE] = !finalResult
        return finalResult
    }

    companion object {
        fun getFiringSpeed(isFarZone: Boolean = false): Pair<Long, Long> {
            if (isFarZone) return farUpDuration to farDownDuration
            return upDuration to downDuration
        }
    }
}
