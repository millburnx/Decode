package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class MotorTester : OpMode() {
    override fun run() {
        val motor = hardwareMap.get(motorName) as DcMotorEx
        val analog = hardwareMap.analogInput[analogName]
        scheduler.schedule(Command() {
            while (!isStarted) {
                sync()
            }
            while (!isStopRequested) {
                motor.power = power
                tel.addData("power", power)
                tel.addData("position", motor.currentPosition)
                tel.addData("velocity", motor.velocity)
                val voltage = analog.voltage
                tel.addData("voltage", voltage)
                tel.addData("normalized voltage", voltage / 3.3)
                tel.addData("360 voltage", voltage / 3.3 * 360)
                tel.update()
                sync()
            }
        })
    }

    companion object {
        @JvmField
        var motorName = "m0"
        @JvmField
        var analogName = "a0"
        @JvmField
        var power = 0.0
    }
}