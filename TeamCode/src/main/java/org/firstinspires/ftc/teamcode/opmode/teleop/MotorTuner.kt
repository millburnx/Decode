package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class MotorTuner
    : OpMode() {
    override fun run() {
//        scheduler.schedule(TeleOpStopper(this))
        val motor = hardwareMap.dcMotor[motorName]
        scheduler.schedule(Command {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                println("setting $motorName to $power - ee")
                motor.power = power
                sync()
            }
        })
    }

    companion object {
        @JvmField
        var motorName = "m0"

        @JvmField
        var power = 0.0
    }
}