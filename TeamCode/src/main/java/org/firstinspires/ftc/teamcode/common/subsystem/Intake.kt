package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.commands.AutoFireSettings
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualMotor
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class Intake(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Intake") {
    val motor = ManualMotor(opMode.hardwareMap, "m3e")
    var power = 0.0
    private var timer: ElapsedTime? = null
    private var unjamingTimer: ElapsedTime? = null

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (isTeleop) {
                    val triggerAmount = gp1.current.rightTrigger - gp1.current.leftTrigger
                    if (!jamProtection) {
                        power = triggerAmount
                    } else {
                        if (timer == null && unjamingTimer == null && triggerAmount > 0.1) {
                            timer = ElapsedTime()
                        } else if (triggerAmount <= 0.1) {
                            timer = null
                            unjamingTimer = null
                        }
                        if ((timer?.milliseconds() ?: -1.0) > intakeDuration) {
                            power = -AutoFireSettings.unjamPower
                            timer = null
                            unjamingTimer = ElapsedTime()
                        } else if (unjamingTimer == null) {
                            power = triggerAmount
                        } else if ((unjamingTimer?.milliseconds() ?: -1.0) > AutoFireSettings.unjamDuration) {
                            unjamingTimer = null
                        } else {
                            power = -AutoFireSettings.unjamPower
                        }
                    }
                }
                motor.power = power
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var jamProtection = true

        @JvmField
        var intakeDuration = 1500L
    }
}