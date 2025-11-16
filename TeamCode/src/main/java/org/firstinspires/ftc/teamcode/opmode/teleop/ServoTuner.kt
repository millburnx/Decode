package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class ServoTuner : OpMode() {
    override fun run() {
//        scheduler.schedule(TeleOpStopper(this))
        val servo = hardwareMap.servo[servoName]
        scheduler.schedule(Command {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                servo.position = position
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