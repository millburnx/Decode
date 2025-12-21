package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class ServoTester : OpMode() {
    override fun run() {
        val servo = hardwareMap.servo[servoName]
        scheduler.schedule(Command() {
            while (!isStarted) {
                sync()
            }
            while (!isStopRequested) {
                servo.position = position
                tel.addData("position", position)
                sync()
            }
        })
    }

    companion object {
        @JvmField
        var servoName = "s0"
        @JvmField
        var position = 0.0
    }
}