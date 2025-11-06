package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDController
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualAxon
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.abs

@Configurable
class Uppies(
    opMode: OpMode,
    val flyWheelState: () -> FlyWheel.State,
    var isTeleop: Boolean = false
) : Subsystem("Uppies") {
    val servo = ManualAxon(opMode.hardwareMap, "s0", "a0", threshold = 0.001)
    var state = State.INTAKE_READY
    var rotations = 0;
    val target: Double
        get() = rotations + when (state) {
            State.INTAKE_READY -> IntakeReadyPosition
            State.LOADER_READY -> LoaderReadyPosition
            State.LOADED -> LoadedPosition
        }

    val atTarget: Boolean
        get() = abs(servo.position - target) <= threshold

    val pid = PIDController(kP, kI, kD)

    val teleopState = TeleopData()

    fun prevState() {
        state = if (state == State.LOADED) {
            if (flyWheelState() == FlyWheel.State.INTAKING) {
                State.LOADER_READY
            } else {
                State.INTAKE_READY
            }
        } else {
            if (state == State.INTAKE_READY) {
                rotations--
            }
            State.LOADED
        }
    }

    fun nextState() {
        state = if (state == State.LOADED) {
            if (flyWheelState() == FlyWheel.State.INTAKING) {
                rotations--
                State.LOADER_READY
            } else {
                rotations++
                State.INTAKE_READY
            }
        } else {
            State.LOADED
        }
    }

    fun flywheelSync() {
        if (flyWheelState() == FlyWheel.State.INTAKING && state == State.INTAKE_READY) {
            rotations--
            state = State.LOADER_READY
        }
        if (flyWheelState() != FlyWheel.State.INTAKING && state == State.LOADER_READY) {
            rotations++
            state = State.INTAKE_READY
        }
    }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || !isStopRequested }
            while (!isStopRequested) {
                teleOpControls()

                flywheelSync()
                pid.setPID(kP, kI, kD)
                servo.power = pid.calculate(servo.position, target)

                tel.addData("uppies state", state)
                tel.addData("uppies position", servo.position)
                sync()
            }
        }
    }

    private fun OpMode.teleOpControls() {
        if (isTeleop) {
            val currentLeftBumper = gp1.left_bumper
            val currentRightBumper = gp1.right_bumper
            if (currentLeftBumper && !teleopState.prevLeftBumper) {
                prevState()
            }
            if (currentRightBumper && !teleopState.prevRightBumper) {
                nextState()
            }
            teleopState.prevLeftBumper = currentLeftBumper
            teleopState.prevRightBumper = currentRightBumper
        }
    }

    enum class State {
        INTAKE_READY, LOADER_READY, LOADED,
    }

    data class TeleopData(var prevLeftBumper: Boolean = false, var prevRightBumper: Boolean = false)

    companion object {
        @JvmField
        var kP = 2.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.06

        @JvmField
        var threshold = 0.05

        @JvmField
        var IntakeReadyPosition = 0.125

        @JvmField
        var LoaderReadyPosition = 0.68

        @JvmField
        var LoadedPosition = 0.4
    }
}