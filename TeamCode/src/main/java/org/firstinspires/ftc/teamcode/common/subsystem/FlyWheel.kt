package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.hardware.Encoder
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.abs

@Configurable
class FlyWheel(val opMode: OpMode) : Subsystem("FlyWheel") {
    val leftMotor = ManualMotor(opMode.hardwareMap, leftMotorName, leftMotorReversed)
    val rightMotor = ManualMotor(opMode.hardwareMap, rightMotorName, rightMotorReversed)
    val encoder = Encoder(opMode.hardwareMap, encoderName, encoderReversed)
    val rpm get() = encoder.velocity * toRPM

    var shootingRPM = baseRPM
    val targetRpm: Double
        get() = when (state) {
            FlyWheelState.IDLE -> idleRPM
            FlyWheelState.SHOOTING -> shootingRPM
            FlyWheelState.INTAKING -> intakingRPM
        }

    val atRPM: Boolean
        get() = state == FlyWheelState.IDLE || abs(rpm - targetRpm) < Controller.rpmThreshold

    val pidf = Controller()
    val idlePidf = Controller(false, false)

    var state = FlyWheelState.IDLE

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            with(opMode) {
                if (override) {
                    leftMotor.power = overridePower
                    rightMotor.power = overridePower
                    tel.addData("flywheel | rpm", rpm)
                    return@OpModeLoop
                }

                val power =
                    if (state == FlyWheelState.IDLE) {
                        idlePidf.calculate(rpm, targetRpm, voltageSensor.voltage)
                    } else {
                        pidf.calculate(rpm, targetRpm, voltageSensor.voltage)
                    }

                leftMotor.power = power
                rightMotor.power = power

                tel.addData("flywheel | power", power)
                tel.addData("flywheel | target", targetRpm)
                tel.addData("flywheel | at rpm", atRPM)
                tel.addData("flywheel | rpm", rpm)
            }
        }
    }

    companion object {
        @JvmField
        var override = false

        @JvmField
        var overridePower = 0.0

        @JvmField
        var leftMotorName = "m3e"

        @JvmField
        var rightMotorName = "m2e"

        @JvmField
        var encoderName = "m1e"

        @JvmField
        var leftMotorReversed = true

        @JvmField
        var rightMotorReversed = false

        @JvmField
        var encoderReversed = true

        @JvmField
        var toRPM = 60 / 28.0

        @JvmField
        var idleRPM = 2400.0

        @JvmField
        var baseRPM = 2900.0

        @JvmField
        var intakingRPM = -1000.0
    }

    enum class FlyWheelState {
        IDLE,
        SHOOTING,
        INTAKING;
    }

    @Configurable
    class Controller(val pid: Boolean = true, val voltageCompensation: Boolean = true) {
        val PID = PIDController(kP, kI, kD)
        var FF = SimpleMotorFeedforward(kS, kV, 0.0)

        fun calculate(currentVelocity: Double, targetVelocity: Double, voltage: Double): Double {
            PID.setPID(kP, kI, kD)
            if (FF.ks != kS || FF.kv != kV) {
                FF = SimpleMotorFeedforward(kS, kV, 0.0)
            }

            val pid = if (pid) PID.calculate(currentVelocity, targetVelocity) else 0.0
            val ff = FF.calculate(targetVelocity)

            val voltageComp = if (voltage != 0.0 && voltageCompensation) (12.0 / voltage) else 1.0
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
            var kS = 0.09

            @JvmField
            var kV = 0.00018

            @JvmField
            var rpmThreshold = 200.0
        }
    }
}