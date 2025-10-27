package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.subsystems.AxonCR
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
            if (abs(value - field) > threshold) {
                field = value
                axon.power = value
            }
        }

    val position
        get() = axon.position

    val rawPosition
        get() = axon.rawPosition
}