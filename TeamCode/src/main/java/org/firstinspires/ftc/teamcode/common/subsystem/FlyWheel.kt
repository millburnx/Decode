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
            val leftAtVelocity = abs(targetVelocity + leftMotor.velocity) <= velocityThreshold
            val rightAtVelocity = abs(targetVelocity - rightMotor.velocity) <= velocityThreshold
            return leftAtVelocity && rightAtVelocity
        }

    var shootingVelocity = ShootingVelocity

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                teleOpControls()

                targetVelocity = when (state) {
                    State.IDLE -> 0.0
                    State.SHOOTING -> shootingVelocity // replace with distance based
                    State.INTAKING -> IntakingVelocity
                }

                leftMotor.power = leftController.calculate(-leftMotor.velocity, targetVelocity, voltageSensor.voltage)
                rightMotor.power = rightController.calculate(rightMotor.velocity, targetVelocity, voltageSensor.voltage)

                tel.addData("fw state", state)
                tel.addData("fw-l velocity", -leftMotor.velocity)
                tel.addData("fw-r velocity", rightMotor.velocity)
                tel.addData("shooting velocity", shootingVelocity)
                tel.addData("voltage", voltageSensor.voltage)
                sync()
            }
        }
    }

    private fun OpMode.teleOpControls() {
        if (isTeleop) {
//            println("${gp1.prev.x} -> ${gp1.current.x} | ${gp1.prev} -> ${gp1.current.x}")
            if (gp1.current.x && !gp1.prev.x) {
                state = if (state == State.SHOOTING) State.IDLE else State.SHOOTING
            }
            if (gp1.current.b && !gp1.prev.b) {
                state = if (state == State.INTAKING) State.IDLE else State.INTAKING
            }
        }
    }

    enum class State {
        IDLE,
        SHOOTING,
        INTAKING
    }

    companion object {
        @JvmField
        var ShootingVelocity = 2550.0

        @JvmField
        var IntakingVelocity = -1600.0

        @JvmField
        var velocityThreshold = 50.0
    }
}

@Configurable
class FlyWheelController() {
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
        var kP = 0.001

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var kS = 0.04

        @JvmField
        var kV = 0.00037

        @JvmField
        var voltageWeight = 1.0
    }
}