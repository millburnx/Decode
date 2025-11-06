package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command

abstract class Subsystem(
    val name: String
) {
    init {
        SubsystemManager.subsystems.add(this)
    }

    abstract val run: suspend Command.() -> Unit
    open val cleanup: () -> Unit = {}

    // by lazy to make sure it's created post-overrides
    val command: Command by lazy { Command(name, cleanup, run) }
}