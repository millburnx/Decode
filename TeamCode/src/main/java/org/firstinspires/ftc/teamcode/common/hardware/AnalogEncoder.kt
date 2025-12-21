package org.firstinspires.ftc.teamcode.common.hardware

import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * @param quadratureVelocity: used as a sanity check for wrap arounds
 */
class AnalogEncoder(
    hardwareMap: HardwareMap,
    name: String,
    val reverse: Boolean = false,
    val wrapAroundThreshold: Double = 0.5,
    val quadratureVelocity: (() -> Double)? = null,
) {
    val encoder = hardwareMap.analogInput[name]

    var rawPosition = 0.0
    var rotations = 0
    val position
        get() = rotations + rawPosition

    fun update() {
        val prev = rawPosition
        val raw = encoder.voltage / 3.3
        if (reverse) rawPosition = raw else rawPosition = 1 - raw

        val angleDifference: Double = rawPosition - prev

        // Handle wraparound at 0|1 boundary
        if (quadratureVelocity == null) {
            if (angleDifference < -wrapAroundThreshold) {
                rotations++ // 1 to 0
            } else if (angleDifference > wrapAroundThreshold) {
                rotations-- // 0 to 1
            }
        } else {
            val velo = quadratureVelocity()
            if (angleDifference < -wrapAroundThreshold && velo > 0) {
                rotations++
            } else if (angleDifference > wrapAroundThreshold && velo < 0) {
                rotations--
            }
        }
    }
}