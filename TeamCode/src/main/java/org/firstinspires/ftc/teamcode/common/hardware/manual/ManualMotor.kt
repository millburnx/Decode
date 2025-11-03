package org.firstinspires.ftc.teamcode.common.hardware.manual

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.common.hardware.cached.CachedMotor
import kotlin.math.abs

class ManualMotor(
    hardwareMap: HardwareMap,
    name: String,
    reverse: Boolean = false,
    float: Boolean = true,
    threshold: Double = 0.05
) : CachedMotor(hardwareMap, name, reverse, float, threshold) {
    init {
        ManualManager.motors.add(this)
    }

    override var power = 0.0
        set(value) {
            if (abs(value - field) > threshold) {
                field = value
                pending = true
            }
        }

    private var pending = false

    fun update() {
        if (pending) {
            motor.power = power
            pending = false
        }
    }
}