package org.firstinspires.ftc.teamcode.subsystems

import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

fun motorSetup(motor: DcMotorEx, reverse: Boolean = false, float: Boolean = false) {
    motor.zeroPowerBehavior = if (float) DcMotor.ZeroPowerBehavior.FLOAT else DcMotor.ZeroPowerBehavior.BRAKE
    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
    motor.direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
}

class FlyWheel(opMode: LinearOpMode) : Subsystem("FlyWheel") {
    val left = (opMode.hardwareMap["m1e"] as DcMotorEx).apply { motorSetup(this, reverse = true, float = true) }
    val right = (opMode.hardwareMap["m2e"] as DcMotorEx).apply { motorSetup(this, reverse = true, float = true) }

    var running = false

    override val run: suspend Command.() -> Unit = {
        with (opMode) {
            var wasDown = gamepad1.x
            while (opModeIsActive() && !isStopRequested) {
                val isDown = gamepad1.x
                if (isDown != wasDown && isDown) {
                    running = !running
                }
                wasDown = isDown

                if (running) {
                    left.power = 1.0
                    right.power = 1.0
                } else {
                    left.power = 0.0
                    right.power = 0.0
                }
                sync()
            }
        }
    }

    override val command = Command(this.name,cleanup,run)
}