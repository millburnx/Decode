package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.abs

class FlyWheel(val opMode: OpMode) : Subsystem("FlyWheel") {
    val leftMotor = ManualMotor(opMode.hardwareMap, leftMotorName, leftMotorReversed)
    val rightMotor = ManualMotor(opMode.hardwareMap, rightMotorName, rightMotorReversed)

    val leftRPM get() = leftMotor.velocity * toRPM
    val rightRPM get() = rightMotor.velocity * toRPM
    val rpm get() = (leftRPM + rightRPM) / 2.0

    var shootingRPM = baseRPM
    val targetRpm: Double
        get() = when (state) {
            FlyWheelState.IDLE -> 0.0
            FlyWheelState.SHOOTING -> shootingRPM
            FlyWheelState.INTAKING -> intakingRPM
        }

    val atRPM: Boolean
        get() = state == FlyWheelState.IDLE || abs(rpm - targetRpm) < Controller.rpmThreshold

    val pidf = Controller()

    var state = FlyWheelState.IDLE

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            if (state == FlyWheelState.IDLE) {
                leftMotor.power = 0.0
                rightMotor.power = 0.0
                return@OpModeLoop
            }

            with(opMode) {
                val power = pidf.calculate(rpm, targetRpm, voltageSensor.voltage)
                leftMotor.power = power
                rightMotor.power = power

                tel.addData("flywheel | power", power)
                tel.addData("flywheel | target", targetRpm)
                tel.addData("flywheel | rpm", rpm)
            }
        }
    }

    companion object {
        @JvmField
        var leftMotorName = "m0e"

        @JvmField
        var rightMotorName = "m1e"

        @JvmField
        var leftMotorReversed = false

        @JvmField
        var rightMotorReversed = false

        @JvmField
        var toRPM = 60 / 28.0

        @JvmField
        var baseRPM = 2500.0

        @JvmField
        var intakingRPM = -1000.0
    }

    enum class FlyWheelState {
        IDLE,
        SHOOTING,
        INTAKING;
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

            val voltageComp = if (voltage != 0.0) (12.0 / voltage) else 1.0
            return (pid + ff) * voltageComp
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
            var rpmThreshold = 150.0
        }
    }
}