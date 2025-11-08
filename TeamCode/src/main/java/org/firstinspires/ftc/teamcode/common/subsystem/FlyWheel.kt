package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.abs

@Configurable
class FlyWheel(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("FlyWheel") {
    val leftMotor = ManualMotor(opMode.hardwareMap, "m2e", reverse = true)
    val rightMotor = ManualMotor(opMode.hardwareMap, "m1e", reverse = true)
    val leftController = FlyWheelController()
    val rightController = FlyWheelController()
    var state = State.IDLE
    var targetVelocity = 0.0

    val atVelocity: Boolean
        get() {
            val leftAtVelocity = abs(leftMotor.velocity - targetVelocity) <= velocityThreshold
            val rightAtVelocity = abs(-rightMotor.velocity - targetVelocity) <= velocityThreshold
            return leftAtVelocity && rightAtVelocity
        }

    val teleopState = TeleopData()

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                teleOpControls()

                targetVelocity = when (state) {
                    State.IDLE -> 0.0
                    State.SHOOTING -> ShootingVelocity // replace with distance based
                    State.INTAKING -> IntakingVelocity
                }

                leftMotor.power = leftController.calculate(leftMotor.velocity, targetVelocity, 0.0)
                rightMotor.power = rightController.calculate(-rightMotor.velocity, targetVelocity, 0.0)

                tel.addData("fw state", state)
                tel.addData("fw-l velocity", leftMotor.velocity)
                tel.addData("fw-r velocity", -rightMotor.velocity)
                sync()
            }
        }
    }

    private fun OpMode.teleOpControls() {
        if (isTeleop) {
            val currentX = gp1.x
            val currentB = gp1.b
            if (currentX && !teleopState.prevX) {
                state = if (state == State.SHOOTING) State.IDLE else State.SHOOTING
            }
            if (currentB && !teleopState.prevB) {
                state = if (state == State.INTAKING) State.IDLE else State.INTAKING
            }
            teleopState.prevX = currentX
            teleopState.prevB = currentB
        }
    }

    enum class State {
        IDLE,
        SHOOTING,
        INTAKING
    }

    companion object {
        @JvmField
        var ShootingVelocity = 1600.0

        @JvmField
        var IntakingVelocity = -1600.0

        @JvmField
        var velocityThreshold = 200.0
    }

    data class TeleopData(var prevX: Boolean = false, var prevB: Boolean = false)
}

@Configurable
class FlyWheelController() {
    val PID = PIDController(kP, kI, kD)
    val FF = SimpleMotorFeedforward(kS, kV, 0.0)

    fun calculate(currentVelocity: Double, targetVelocity: Double, voltage: Double): Double {
        val pid = PID.calculate(currentVelocity, targetVelocity)
        val ff = FF.calculate(targetVelocity)
        val voltageCompensation = (12.0 / voltage)
        val weightedVoltageCompensation = 1 + (1 - voltageCompensation) * voltageWeight

        return (pid + ff) * weightedVoltageCompensation
    }

    companion object {
        @JvmField
        var kP = 0.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var kS = 0.0

        @JvmField
        var kV = 0.0

        @JvmField
        var voltageWeight = 0.0
    }
}