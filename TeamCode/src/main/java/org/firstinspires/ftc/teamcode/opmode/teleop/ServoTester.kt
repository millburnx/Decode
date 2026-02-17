package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@TeleOp
@Configurable
class ServoTester : OpMode() {
    override fun run() {
        val s1 = hardwareMap.servo[servoName]

        s1.direction = if (servoReversed) Servo.Direction.REVERSE else Servo.Direction.FORWARD

        scheduler.schedule(Command {
            OpModeLoop(this@ServoTester) {
                s1.position = position
            }
        })
    }

    companion object {
        @JvmField
        var servoName = "s0"

        @JvmField
        var servoReversed = true

        @JvmField
        var position = 0.5
    }
}