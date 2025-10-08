package org.firstinspires.ftc.teamcode.subsystems

import com.millburnx.cmdx.Command

abstract class Subsystem(name: String) {
    open fun init() {}
    open val run: suspend Command.() -> Unit = {}
    open val cleanup: () -> Unit = {}
    abstract val command: Command
}