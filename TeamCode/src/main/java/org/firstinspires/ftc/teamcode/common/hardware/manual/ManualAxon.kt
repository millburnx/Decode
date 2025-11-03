package org.firstinspires.ftc.teamcode.common.hardware.manual

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.common.hardware.cached.CachedAxon
import kotlin.math.abs

class ManualAxon(
    hardwareMap: HardwareMap,
    name: String,
    encoderName: String,
    reverse: Boolean = false,
    encoderReverse: Boolean = false,
    threshold: Double = 0.05
) : CachedAxon(hardwareMap, name, encoderName, reverse, encoderReverse, threshold) {
    init {
        ManualManager.axons.add(this)
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
            axon.power = power
            pending = false
        }
        axon.updatePosition()
    }
}