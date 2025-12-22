package org.firstinspires.ftc.teamcode.common.hardware.manual

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.common.hardware.cached.CachedServo
import kotlin.math.abs

class ManualServo(
    hardwareMap: HardwareMap,
    name: String,
    reverse: Boolean = false,
    threshold: Double = 0.05
) : CachedServo(hardwareMap, name, reverse, threshold) {
    init {
        ManualManager.servos.add(this)
    }

    override var position = 0.0
        set(value) {
            val clamped = value.coerceIn(-1.0, 1.0)
            if (abs(clamped - field) > threshold) {
                field = clamped
                pending = true
            }
        }

    private var pending = false

    fun update() {
        if (pending) {
            servo.position = position
            pending = false
        }
    }
}