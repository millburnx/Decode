package org.firstinspires.ftc.teamcode.common.subsystem

import com.arcrobotics.ftclib.controller.PIDController
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualAxon
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class Uppies(
    opMode: OpMode,
    val flyWheelState: () -> FlyWheel.State,
    var isTeleop: Boolean = false
) : Subsystem("Uppies") {
    val leftServo = ManualAxon(opMode.hardwareMap, "s0", "a0", threshold = 0.01)
    var leftState = LeftState.INTAKE_READY
    var leftRotations = 0;
    val leftTarget: Double
        get() = leftRotations + when (leftState) {
            LeftState.INTAKE_READY -> IntakeReadyPosition
            LeftState.LOADER_READY -> LoaderReadyPosition
            LeftState.LOADED -> LoadedPosition
        }
    val leftPID = PIDController(kP, kI, kD)

    val teleopState = TeleopData()

    fun prevState() {
        leftState = if (leftState == LeftState.LOADED) {
            if (flyWheelState() == FlyWheel.State.INTAKING) {
                LeftState.LOADER_READY
            } else {
                LeftState.INTAKE_READY
            }
        } else {
            if (leftState == LeftState.INTAKE_READY) {
                leftRotations--
            }
            LeftState.LOADED
        }
    }

    fun nextState() {
        leftState = if (leftState == LeftState.LOADED) {
            if (flyWheelState() == FlyWheel.State.INTAKING) {
                leftRotations--
                LeftState.LOADER_READY
            } else {
                leftRotations++
                LeftState.INTAKE_READY
            }
        } else {
            LeftState.LOADED
        }
    }

    fun flywheelSync() {
        if (flyWheelState() == FlyWheel.State.INTAKING && leftState == LeftState.INTAKE_READY) {
            leftRotations--
            leftState = LeftState.LOADER_READY
        }
        if (flyWheelState() != FlyWheel.State.INTAKING && leftState == LeftState.LOADER_READY) {
            leftRotations++
            leftState = LeftState.INTAKE_READY
        }
    }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || !isStopRequested }
            while (!isStopRequested) {
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
                flywheelSync()
                leftPID.setPID(kP, kI, kD)
                leftServo.power = leftPID.calculate(leftServo.position, leftTarget)
                tel.addData("left state", leftState)
                tel.addData("left position", leftServo.position)
                sync()
            }
        }
    }

    enum class LeftState {
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
        var IntakeReadyPosition = 0.125

        @JvmField
        var LoaderReadyPosition = 0.68

        @JvmField
        var LoadedPosition = 0.4
    }
}