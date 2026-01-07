package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class ColorTester : OpMode() {
    override fun run() {
        val sensor = hardwareMap.colorSensor[sensorName]
        val sensor2 = hardwareMap[sensorName2] as RevColorSensorV3
        scheduler.schedule(Command() {
            while (!isStarted) {
                sync()
            }
            while (!isStopRequested) {
                tel.addData("r", sensor.red())
                tel.addData("g", sensor.green())
                tel.addData("b", sensor.blue())
                tel.addData("r1", sensor2.normalizedColors.red)
                tel.addData("g1", sensor2.normalizedColors.green)
                tel.addData("b1", sensor2.normalizedColors.blue)
                sync()
            }
        })
    }

    companion object {
        @JvmField
        var sensorName = "c2"
        @JvmField
        var sensorName2 = "c1"
    }
}