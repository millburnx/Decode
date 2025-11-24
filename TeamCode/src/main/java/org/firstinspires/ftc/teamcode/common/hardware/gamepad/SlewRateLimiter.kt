package org.firstinspires.ftc.teamcode.common.hardware.gamepad

object SlewRateLimiter {
    fun limit(current: Double, target: Double, maxRate: Double): Double {
        val rate = target - current
        return current + rate.coerceIn(-maxRate, maxRate)
    }
}