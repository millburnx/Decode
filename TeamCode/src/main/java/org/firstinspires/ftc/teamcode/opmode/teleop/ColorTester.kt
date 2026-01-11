package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Sorter
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class ColorTester : OpMode() {
    override fun run() {
        val Sorter = Sorter(this)
//        scheduler.schedule(Command() {
//            WaitFor { isStarted }
//            while (!isStopRequested) {
//                sync()
//            }
//        })
    }

    companion object {
        @JvmField
        var sensorName = "c2"
        @JvmField
        var sensorName2 = "c1"
    }
}