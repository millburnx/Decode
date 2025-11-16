package org.firstinspires.ftc.teamcode.common.hardware.cached

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.common.hardware.motorSetup
import kotlin.math.abs

open class CachedMotor(
    val hardwareMap: HardwareMap,
    val name: String,
    val reverse: Boolean = false,
    float: Boolean = true,
    val threshold: Double = 0.05
) {
    val motor = (hardwareMap[name] as DcMotorEx).apply {
        motorSetup(this, reverse = reverse, float = float)
    }

    open var power = 0.0
        set(value) {
            val clamped = value.coerceIn(-1.0, 1.0)
            if (abs(clamped - field) > threshold) {
                field = clamped
                motor.power = clamped
            }
        }

    val position
        get() = motor.currentPosition

    var mode = motor.mode
        set(value) {
            if (field != value) {
                field = value
                motor.mode = value
            }
        }

    var float = float
        set(value) {
            if (field != value) {
                field = value
                if (value) {
                    motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                } else {
                    motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                }
            }
        }

    val velocity
        get() = motor.velocity
}