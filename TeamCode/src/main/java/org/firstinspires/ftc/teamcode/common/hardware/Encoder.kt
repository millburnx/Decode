package org.firstinspires.ftc.teamcode.common.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

class Encoder(
    hardwareMap: HardwareMap,
    name: String,
    val reverse: Boolean = false
) {
    private val motor = (hardwareMap[name] as DcMotorEx).apply {
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
    }

    val position
        get() = motor.currentPosition * if (reverse) -1.0 else 1.0

    val velocity
        get() = motor.velocity * if (reverse) -1.0 else 1.0
}