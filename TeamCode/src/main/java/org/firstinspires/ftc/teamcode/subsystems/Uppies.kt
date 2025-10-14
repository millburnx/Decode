package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.absoluteValue

class AxonCR(hardwareMap: HardwareMap, name: String, encoderName: String, reverse: Boolean = false) {
    val servo = hardwareMap.crservo[name].apply {
        direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    }
    var power
        get() = servo.power
        set(power) {
            servo.power = power
        }

    val encoder = hardwareMap.analogInput[encoderName]
    var rawPosition: Double = 0.0
    var rotations: Int = 0
    val position
        get() = rotations + rawPosition

    fun updatePosition(): Double {
        val oldPosition = rawPosition
        rawPosition = encoder.voltage / 3.3

        val angleDifference: Double = rawPosition - oldPosition
        val threshold = 0.5

        // Handle wraparound at 0|1 boundary
        if (angleDifference < -threshold) {
            rotations++ // 1 to 0
        } else if (angleDifference > threshold) {
            rotations-- // 0 to 1
        }

        return position
    }
}

class SingleP(public var kP: Double) {
    fun calc(target: Double, current: Double): Double {
        var error = target - current
        if (error < 0.0 && error.absoluteValue > 20.0 / 360.0) {
            error = (target + 1) - current
        }
        return kP * error
    }
}

@Config
class Uppies(opMode: LinearOpMode) : Subsystem("Uppies") {
//    val left = AxonCR(opMode.hardwareMap, "s0", "a0")
//    val leftController = BasicPID(PIDCoefficients(kP, kI, kD))

    val right = AxonCR(opMode.hardwareMap, "s0", "a0", true)
    val left = AxonCR(opMode.hardwareMap, "s1", "a1")

    var leftState: Positions = Positions.OPEN;
    var rightState: Positions = Positions.OPEN;


    val states = listOf<Positions>(
        Positions.OPEN,
        Positions.BOTTOM,
        Positions.TOP,
    )

    val controller = SingleP(kP)

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            var prevLeft = gamepad1.left_bumper
            var prevRight = gamepad1.right_bumper
            while (opModeIsActive() && !isStopRequested) {
                left.updatePosition()
                right.updatePosition()
                controller.kP = kP
                // update state
                val newLeft = opMode.gamepad1.left_bumper
                if (prevLeft != newLeft && newLeft) {
                    // button pressed
                    leftState = states[(states.indexOf(leftState) + 1) % states.size]
                }
                val newRight = opMode.gamepad1.right_bumper
                if (prevRight != newRight && newRight) {
                    // button pressed
                    rightState = states[(states.indexOf(rightState) + 1) % states.size]
                }
                prevLeft = newLeft
                prevRight = newRight

                left.power = controller.calc(leftState.getPosition(true), left.rawPosition)
                right.power = controller.calc(rightState.getPosition(false), right.rawPosition)

                opMode.telemetry.addData("left state", leftState)
                opMode.telemetry.addData("right state", rightState)
                opMode.telemetry.addData("left pos", left.rawPosition)
                opMode.telemetry.addData("right pos", right.rawPosition)
                opMode.telemetry.update()
                sync()
            }
        }
    }

    override val command = Command(this.name,cleanup,run)

    enum class Positions {
        OPEN, BOTTOM, TOP;

        fun getPosition(left: Boolean): Double = when (this) {
            OPEN -> if (left) openLeft else openRight
            BOTTOM -> if (left) bottomLeft else bottomRight
            TOP -> if (left) topLeft else topRight
        }
    }

    companion object {
        @JvmField
        var kP = 0.4

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var openLeft = 0.0 / 360.0

        @JvmField
        var bottomLeft = 0.0 / 360.0

        @JvmField
        var topLeft = 0.0 / 360.0

        @JvmField
        var openRight = 0.0 / 360.0

        @JvmField
        var bottomRight = 0.0 / 360.0

        @JvmField
        var topRight = 0.0 / 360.0
    }
}