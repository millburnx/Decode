package org.firstinspires.ftc.teamcode.opmode.test.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleOpDrive
import org.firstinspires.ftc.teamcode.common.subsystem.Turret
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp(group = "test")
class TurretTester : OpMode() {
    override fun run() {
        val drive = TeleOpDrive(this, true)

        val turret = Turret(
            this,
            { drive.pose.heading },
            { drive.velocity.heading },
            { voltageSensor.voltage }
        )

        scheduler.schedule(
            Command("teleop loop")
            {
                OpModeLoop(this@TurretTester) {
                    turret.target = target
                    turret.targetingMode =
                        if (fieldCentric) Turret.TargetingMode.GLOBAL else Turret.TargetingMode.RELATIVE
                }
            })
    }

    companion object {
        @JvmField
        var target = 180.0

        @JvmField
        var fieldCentric = false
    }
}