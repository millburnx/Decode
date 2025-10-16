package org.firstinspires.ftc.teamcode.subsystems

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class AxonCR(hardwareMap: HardwareMap, name: String, encoderName: String, reverse: Boolean = false, val encoderReverse: Boolean = false) {
    val servo: CRServo = hardwareMap.crservo[name].apply {
        direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    }
    var power
        get() = servo.power
        set(power) {
            servo.power = power
        }

    val encoder = hardwareMap.analogInput[encoderName]
    var rawPosition: Double = 0.0 //kedaar wuz here
    var rotations: Int = 0 //kedaar wuz also here
    val position
        get() = rotations + rawPosition

    fun updatePosition(): Double {
        val oldPosition = rawPosition
        val raw = encoder.voltage / 3.3
        if (encoderReverse) rawPosition = raw else rawPosition = 1 - raw

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

@Config
class Uppies(opMode: LinearOpMode, tel: MultipleTelemetry) : Subsystem("Uppies") {
//    val left = AxonCR(opMode.hardwareMap, "s0", "a0")
//    val leftController = BasicPID(PIDCoefficients(kP, kI, kD))

    val right = AxonCR(opMode.hardwareMap, "s0", "a0", reverse = true, encoderReverse = true)
    val left = AxonCR(opMode.hardwareMap, "s1", "a1", reverse = true)

    var leftState: Positions = Positions.OPEN;
    var rightState: Positions = Positions.OPEN;

    var state: States = States.OPEN


    val positions = listOf<Positions>(
        Positions.OPEN,
        Positions.BOTTOM,
        Positions.TOP,
    )

    val states = listOf<States>(
        States.OPEN,
        States.ONE_BALL,
        States.TWO_BALL,
        States.SHOOT_ONE,
        States.LOAD_TWO,
        States.THIRD_BALL,
        States.SHOOT_TWO,
        States.LOAD_THREE,
    )

    val pidLeft = BasicPID(PIDCoefficients(kP, kI, kD))
    val pidRight = BasicPID(PIDCoefficients(kP, kI, kD))

    var leftRotations = 0
    var rightRotations = 0

    fun nextLeft() {
        if (leftState == positions.last()) leftRotations++
        leftState = positions[(positions.indexOf(leftState) + 1) % positions.size]
    }

    fun nextRight() {
        if (rightState == positions.last()) rightRotations++
        rightState = positions[(positions.indexOf(rightState) + 1) % positions.size]
    }

    fun next() {
        state = states[(states.indexOf(state) + 1) % states.size]
        when (state) {
            States.OPEN -> {
                nextRight()
            }
            States.ONE_BALL -> {
                nextRight()
                nextRight()
            }
            States.TWO_BALL -> {
                nextLeft()
            }
            States.SHOOT_ONE -> {
                nextRight()
            }
            States.LOAD_TWO -> {
                nextLeft()
            }
            States.THIRD_BALL -> {
                nextRight()
            }
            States.SHOOT_TWO -> {
                nextLeft()
            }
            States.LOAD_THREE -> {
                nextRight()
            }
        }
    }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            var prevButton = gamepad1.right_bumper
            while (opModeIsActive() && !isStopRequested) {
                left.updatePosition()
                right.updatePosition()

                val newButton = gamepad1.right_bumper
                if (!prevButton && newButton) {
                    next()
                }
                prevButton = newButton

                val leftTarget = leftRotations + leftState.getPosition(true)
                val rightTarget = rightRotations + rightState.getPosition(false)

                left.power = pidLeft.calculate(leftTarget, left.position)
                right.power = pidRight.calculate(rightTarget, right.position)

                tel.addData("state", state)
                tel.addData("left state", leftState)
                tel.addData("right state", rightState)
                tel.addData("left target", leftTarget)
                tel.addData("right target", rightTarget)
                tel.addData("left raw pos", left.rawPosition)
                tel.addData("right raw pos", right.rawPosition)
                tel.addData("left pos", left.position)
                tel.addData("right pos", right.position)
                sync()
            }
        }
    }

    override val command = Command(this.name,cleanup,run)

    enum class States {
        OPEN,
        ONE_BALL,
        TWO_BALL,
        SHOOT_ONE,
        LOAD_TWO,
        THIRD_BALL,
        SHOOT_TWO,
        LOAD_THREE,
    }
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
        var kP = 5.0 * 0.8

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 5.0 * 0.1

        @JvmField
        var openLeft = 0.075

        @JvmField
        var bottomLeft = 0.175

        @JvmField
        var topLeft = 0.44

        @JvmField
        var openRight = 0.075

        @JvmField
        var bottomRight = 0.175

        @JvmField
        var topRight = 0.44
    }
}