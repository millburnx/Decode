package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Turret
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class TurretTester : OpMode() {
    override fun run() {
        val turret = Turret(this)

        scheduler.schedule(Command("teleop loop") {
            OpModeLoop(this@TurretTester) {
                turret.target = target
            }
        })
    }

    companion object {
        @JvmField
        var target = 180.0
    }
}