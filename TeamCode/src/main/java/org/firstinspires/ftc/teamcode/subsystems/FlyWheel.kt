package org.firstinspires.ftc.teamcode.subsystems

import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

fun motorSetup(motor: DcMotorEx) {
    motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
    motor.direction = DcMotorSimple.Direction.FORWARD
}

class FlyWheel(opMode: LinearOpMode) : Subsystem("FlyWheel") {
    val left = (opMode.hardwareMap["m0"] as DcMotorEx).apply { motorSetup(this) }
    val right = (opMode.hardwareMap["m1"] as DcMotorEx).apply { motorSetup(this) }

    var power: Double = 0.0

    override val run: suspend Command.() -> Unit = {
        with (opMode) {
            while (opModeIsActive() && !isStopRequested) {
                power = gamepad1.left_stick_x.toDouble()

                left.power = power
                right.power = power
                sync()
            }
        }
    }

    override val command = Command("FlyWheel",cleanup,run)
}