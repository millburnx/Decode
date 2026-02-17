package org.firstinspires.ftc.teamcode.opmode.test

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.util.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.hardware.fromPedro
import org.firstinspires.ftc.teamcode.common.subsystem.Turret
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants

@Configurable
@TeleOp
class TurretTester : OpMode() {
    override fun run() {
        val pedro = Constants.createManualFusionFollower(hardwareMap, { deltaTime })

        val turret = Turret(this) { Pose2d.fromPedro(pedro.pose).heading }

        scheduler.schedule(Command("teleop loop") {
            pedro.update()
            pedro.startTeleopDrive(false)
            OpModeLoop(this@TurretTester) {
                pedro.update()
                pedro.setTeleOpDrive(
                    -gp1.current.leftJoyStick.y,
                    -gp1.current.leftJoyStick.x,
                    -gp1.current.rightJoyStick.x,
                    true
                )
                turret.target = target
                turret.targetingMode = if (fieldCentric) Turret.TargetingMode.GLOBAL else Turret.TargetingMode.RELATIVE
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