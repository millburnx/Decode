package org.firstinspires.ftc.teamcode.common.hardware.manual

object ManualManager {
    val motors = mutableListOf<ManualMotor>()
    val axons = mutableListOf<ManualAxon>()

    fun init() {
        motors.clear()
        axons.clear()
    }

    fun update() {
        motors.forEach { it.update() }
        axons.forEach { it.update() }
    }
}