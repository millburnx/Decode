package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.lerp
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel.Controller.Companion.rpmThreshold
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.abs

@Configurable
class FlyWheel(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Shooter") {
    // no clue why it breaks otherwise
    val motor = ManualMotor(opMode.hardwareMap, motorName, reverse = motorReversed, float = true).motor
    val velocity
        get() = motor.velocity

    val rpm
        get() = velocity * 60.0 / 28.0

    val atVelocity
        get() = abs(this@FlyWheel.targetRpm - rpm) <= rpmThreshold

    var targetRpm = 0.0
    var enabled = false

    val controller = Controller()

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
//                this@FlyWheel.targetRpm = FlyWheel.targetRpm
                if (enabled) {
                    val power = controller.calculate(rpm, this@FlyWheel.targetRpm, voltageSensor.voltage)
                    motor.power = power
                    tel.addData("flywheel | at velocity", atVelocity)
                } else {
                    motor.power = 0.0
                    tel.addData("flywheel | at velocity", true)
                }
                tel.addData("flywheel | power", motor.power)
                tel.addData("flywheel | target", targetRpm)
                tel.addData("flywheel | rpm", motor.velocity * 60.0 / 28.0) // 28 ppr @ 6k rpm
                sync()
            }
        }
    }

    @Configurable
    class Controller {
        val PID = PIDController(kP, kI, kD)
        var FF = SimpleMotorFeedforward(kS, kV, 0.0)

        fun calculate(currentVelocity: Double, targetVelocity: Double, voltage: Double): Double {
            PID.setPID(kP, kI, kD)
            if (FF.ks != kS || FF.kv != kV) {
                FF = SimpleMotorFeedforward(kS, kV, 0.0)
            }

            val pid = PID.calculate(currentVelocity, targetVelocity)
            val ff = FF.calculate(targetVelocity)

            val voltageCompensation = if (voltage != 0.0) (12.0 / voltage) else 1.0
            val weightedVoltageCompensation = 1 - (1 - voltageCompensation) * voltageWeight

            return (pid + ff) * weightedVoltageCompensation
        }

        companion object {
            @JvmField
            var kP = 0.0005

            @JvmField
            var kI = 0.0

            @JvmField
            var kD = 0.0

            @JvmField
            var kS = 0.1

            @JvmField
            var kV = 0.00025

            @JvmField
            var voltageWeight = 1.0

            @JvmField
            var rpmThreshold = 150.0
        }
    }

    companion object {
        @JvmField
        var motorName = "m1e"

        @JvmField
        var motorReversed = false

        @JvmField
        var override = true

        val LUT: Map<Double, Pair<Double, Double>> = mapOf(
            32.0 to Pair(1600.0,0.0),
            40.0 to Pair(1700.0, 0.25),
            48.0 to Pair(1800.0, 0.5),
            54.0 to Pair(1900.0, 0.7),
            60.0 to Pair(2000.0, 0.85),
            66.0 to Pair(2100.0, 1.0),
            72.0 to Pair(2150.0, 1.0),
            84.0 to Pair(2250.0, 1.0)
        )

        @JvmField
        var intakeRPM = -500.0

        fun getSettings(distance: Double): Pair<Double, Double> {
            val keys = LUT.keys.sorted()
            if (distance < keys.first()) return LUT[keys.first()]!!
            if (distance > keys.last()) return LUT[keys.last()]!!
            val lowerDist  = keys.last { it <= distance }
            val upperDist = keys.first { it >= distance }
            val lower = LUT[lowerDist]!!
            val upper = LUT[upperDist]!!

            val t = (distance - lowerDist) / (upperDist - lowerDist)
            return lerp(lower.first, upper.first, t) to lerp(lower.second, upper.second, t)
        }
    }
}

