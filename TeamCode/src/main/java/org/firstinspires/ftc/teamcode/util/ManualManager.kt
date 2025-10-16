package org.firstinspires.ftc.teamcode.util

object ManualManager {
    val motors = mutableListOf<ManualMotor>()

    fun init() {
        motors.clear()
    }

    fun update() {
        motors.forEach { it.update() }
    }
}