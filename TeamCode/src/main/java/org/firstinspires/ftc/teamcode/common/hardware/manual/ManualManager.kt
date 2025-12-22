package org.firstinspires.ftc.teamcode.common.hardware.manual

object ManualManager {
    val motors = mutableListOf<ManualMotor>()
    val servos = mutableListOf<ManualServo>()
    val axons = mutableListOf<ManualAxon>()

    fun init() {
        motors.clear()
        servos.clear()
        axons.clear()
    }

    fun update() {
        motors.forEach { it.update() }
        servos.forEach { it.update() }
        axons.forEach { it.update() }
    }
}