package org.firstinspires.ftc.teamcode.common.subsystem.teleop

import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.subsystem.AutoAdjust
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.subsystem.Turret
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

class TeleopTurretManager(
    val opMode: OpMode,
    val turret: Turret,
    val autoAdjust: AutoAdjust,
    val isRed: () -> Boolean,
    val isManual: () -> Boolean,
) : Subsystem("Teleop Turret Manager") {
    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            val joystick = opMode.gp2.current.rightJoyStick
            if (isManual()) {
                if (joystick.magnitude > 0.5) turret.setGlobal() else turret.setRelative()
            }
            turret.target =
                when {
                    // x- both the auto control idle and the uncontrolled turret is 180 degrees
                    turret.isRelative && autoAdjust.enabled -> if (isRed()) 90.0 else 270.0

                    turret.isRelative && !autoAdjust.enabled -> 180.0

                    isManual() -> joystick.degrees

                    else -> autoAdjust.turretAngle
                }
        }
    }
}
