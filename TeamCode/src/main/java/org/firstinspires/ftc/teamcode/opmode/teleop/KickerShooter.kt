package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class KickerShooter : OpMode() {
    override fun run() {
        val motor = hardwareMap.dcMotor[motorName]
        val servo = hardwareMap.servo[servoName]
        scheduler.schedule(Command() {
            while (!isStarted) {
                sync()
            }
            while (!isStopRequested) {
                motor.power = power
                servo.position = position
                sync()
            }
        })
    }

    companion object {
        @JvmField
        var motorName = "m0"
        @JvmField
        var servoName = "s0"
        @JvmField
        var power = 0.0
        @JvmField
        var position = 0.0
    }
}