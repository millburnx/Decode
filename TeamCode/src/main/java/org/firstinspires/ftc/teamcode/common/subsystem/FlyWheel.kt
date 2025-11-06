package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class FlyWheel(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("FlyWheel") {
    val leftMotor = ManualMotor(opMode.hardwareMap, "m2e", reverse = true)
    val rightMotor = ManualMotor(opMode.hardwareMap, "m1e", reverse = true)
    var state = State.IDLE
    var power = 0.0

    val atVelocity: Boolean
        get() {
            return leftMotor.velocity >= targetVelocity && -rightMotor.velocity >= targetVelocity
        }

    val teleopState = TeleopData()

    override val run: suspend Command.() -> Unit = {
        with (opMode) {
            WaitFor { isStarted || !isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    val currentX = gamepad1.x
                    val currentB = gamepad1.b
                    if (currentX && !teleopState.prevX) {
                        state = if (state == State.SHOOTING) State.IDLE else State.SHOOTING
                    }
                    if (currentB && !teleopState.prevB) {
                        state = if (state == State.INTAKING) State.IDLE else State.INTAKING
                    }
                    teleopState.prevX = currentX
                    teleopState.prevB = currentB
                }
                power = when (state) {
                    State.IDLE -> 0.0
                    State.SHOOTING -> ShootingVelocity
                    State.INTAKING -> IntakingVelocity
                }
                leftMotor.power = power
                rightMotor.power = power
                tel.addData("fw state", state)
                tel.addData("fw-l velocity", leftMotor.velocity)
                tel.addData("fw-r velocity", -rightMotor.velocity)
                sync()
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
        var ShootingVelocity = 0.7
        @JvmField
        var IntakingVelocity = -1.0
        @JvmField
        var targetVelocity = 100.0
    }

    data class TeleopData(var prevX: Boolean = false, var prevB: Boolean = false)
}