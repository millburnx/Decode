package org.firstinspires.ftc.teamcode.subsystems

import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class AxonCR(hardwareMap: HardwareMap, name: String, encoderName: String) {
    val servo = hardwareMap.crservo[encoderName].apply {
        direction = DcMotorSimple.Direction.REVERSE
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

class Uppies(opMode: LinearOpMode) : Subsystem("Uppies") {
//    val left = AxonCR(opMode.hardwareMap, "s0", "a0")
//    val leftController = BasicPID(PIDCoefficients(kP, kI, kD))

    val left = opMode.hardwareMap.servo["s0"].apply { direction = Servo.Direction.FORWARD}

    var state: Positions = Positions.OPEN;

    var prevButton = opMode.gamepad1.right_bumper

    val states = listOf<Positions>(
        Positions.OPEN,
        Positions.ONE_LOADED,
        Positions.TWO_LOADED,
        Positions.ONE_SHOT,
        Positions.TWO_SHOT,
        Positions.THREE_SHOT
    )

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            while (opModeIsActive() && !isStopRequested) {
                // update state
                val button = opMode.gamepad1.right_bumper
                if (prevButton != button && button) {
                    // button pressed
                    state = states[(states.indexOf(state) + 1) % states.size]
                }
                prevButton = button
                left.position = state.getPosition()

//                left.updatePosition()
//                left.power = leftController.calculate(state.getPosition(), left.position)
                opMode.telemetry.addData("State", state)
                opMode.telemetry.update()
                sync()
            }
        }
    }

    override val command = Command("Uppies",cleanup,run)

    enum class Positions {
        OPEN, ONE_LOADED, TWO_LOADED, ONE_SHOT, TWO_SHOT, THREE_SHOT;

        fun getPosition(): Double = when (this) {
            Positions.OPEN -> open
            Positions.ONE_LOADED -> one_loaded
            Positions.TWO_LOADED -> two_loaded
            Positions.ONE_SHOT -> one_shot
            Positions.TWO_SHOT -> two_shot
            Positions.THREE_SHOT -> three_shot
        }
    }

    companion object {
        @JvmField
        var kP = 0.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var open = 0.0 / 360.0

        @JvmField
        var one_loaded = 60.0 / 360.0

        @JvmField
        var two_loaded = 120.0 / 360.0

        @JvmField
        var one_shot = 180.0 / 360.0

        @JvmField
        var two_shot = 240 / 360.0

        @JvmField
        var three_shot = 350 / 360.0
    }
}