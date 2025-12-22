package org.firstinspires.ftc.teamcode.common.hardware.cached

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import kotlin.math.abs

open class CachedServo(
    val hardwareMap: HardwareMap,
    val name: String,
    val reverse: Boolean = false,
    val threshold: Double = 0.05
) {
    val servo = (hardwareMap[name] as ServoImplEx).apply {
        direction = if (reverse) Servo.Direction.REVERSE else Servo.Direction.FORWARD
    }

    open var position = 0.0
        set(value) {
            val clamped = value.coerceIn(0.0, 1.0)
            if (abs(clamped - field) > threshold) {
                field = clamped
                servo.position = clamped
            }
        }
}