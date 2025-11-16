package org.firstinspires.ftc.teamcode.common.hardware.cached

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.common.hardware.AxonCR
import kotlin.math.abs

open class CachedAxon(
    val hardwareMap: HardwareMap,
    val name: String,
    val encoderName: String,
    val reverse: Boolean = false,
    val encoderReverse: Boolean = false,
    val threshold: Double = 0.05
) {
    val axon = AxonCR(hardwareMap, name, encoderName, reverse, encoderReverse)

    open var power = 0.0
        set(value) {
            val clamped = value.coerceIn(-1.0, 1.0)
            if (abs(clamped - field) > threshold) {
                field = clamped
                axon.power = clamped
            }
        }

    val position
        get() = axon.position

    val rawPosition
        get() = axon.rawPosition
}