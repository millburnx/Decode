package org.firstinspires.ftc.teamcode.common.hardware

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class AxonCR(
    hardwareMap: HardwareMap,
    name: String,
    encoderName: String,
    reverse: Boolean = false,
    val encoderReverse: Boolean = false
) {
    val servo: CRServo = hardwareMap.crservo[name].apply {
        direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    }
    var power
        get() = servo.power
        set(power) {
            servo.power = power
        }

    val encoder = hardwareMap.analogInput[encoderName]
    var rawPosition: Double = 0.0 //kedaar wuz here
    var rotations: Int = 0 //kedaar wuz also here
    val position
        get() = rotations + rawPosition

    fun updatePosition(): Double {
        val oldPosition = rawPosition
        val raw = encoder.voltage / 3.3
        if (encoderReverse) rawPosition = raw else rawPosition = 1 - raw

        val angleDifference: Double = rawPosition - oldPosition
        val threshold = 0.5

        // Handle wraparound at 0|1 boundary
        if (angleDifference < -threshold) {
            rotations++ // 1 to 0
        } else if (angleDifference > threshold) {
            rotations-- // 0 to 1
        }

        return position
    }
}
