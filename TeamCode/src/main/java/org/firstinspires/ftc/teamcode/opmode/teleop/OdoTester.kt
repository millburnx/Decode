package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class OdoTester : OpMode() {
    override fun run() {
        val odo = hardwareMap["pinpoint"] as GoBildaPinpointDriver
        scheduler.schedule(Command() {
            while (!isStarted) {
                sync()
            }
            while (!isStopRequested) {
                odo.update()
                tel.addData("r", odo.position)
                sync()
            }
        })
    }
}