package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.util.ManualMotor

fun motorSetup(motor: DcMotorEx, reverse: Boolean = false, float: Boolean = false) {
    motor.zeroPowerBehavior = if (float) DcMotor.ZeroPowerBehavior.FLOAT else DcMotor.ZeroPowerBehavior.BRAKE
    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
    motor.direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
}

@Configurable
class FlyWheel(opMode: LinearOpMode) : Subsystem("FlyWheel") {
    val left = ManualMotor(opMode.hardwareMap, "m2e", reverse = true)
    val right = ManualMotor(opMode.hardwareMap, "m1e", reverse = true)

    var running = false
    var power = FarPower;

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            var wasDown = gamepad1.x
            while (opModeIsActive() && !isStopRequested) {
                val isDown = gamepad1.x
                if (isDown != wasDown && isDown) {
                    running = !running
                }
                wasDown = isDown
                if (running) {
                    left.power = power
                    right.power = power
                } else {
                    left.power = 0.0
                    right.power = 0.0
                }
                opMode.telemetry.addData("fw running", running)
                opMode.telemetry.addData("fw power", power)
                sync()
            }
        }
    }

    override val command = Command(this.name, cleanup, run)

    companion object {
        @JvmField
        var FarPower: Double = 1.0
        @JvmField
        var ClosePower: Double = .7
    }
}