package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.runtimeGroups.CommandScheduler

object SubsystemManager {
    val subsystems = mutableListOf<Subsystem>()

    fun init() {
        subsystems.clear()
    }

    fun registerAll(scheduler: CommandScheduler) {
        subsystems.forEach { subsystem ->
            scheduler.schedule(subsystem.command)
        }
    }
}