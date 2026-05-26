package org.firstinspires.ftc.teamcode.common.subsystem.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop.Companion.outtakeBurstDuration
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop.Companion.outtakeBurstPower
import kotlin.math.absoluteValue

@Configurable
class TeleopIntakeManager(val opMode: OpMode, val intake: Intake, val sorter: Sorter) :
    Subsystem("Teleop Intake Manager") {

    override val run: suspend Command.() -> Unit = {
        var lastIntake = System.currentTimeMillis()

        OpModeLoop(opMode) {
            val now = System.currentTimeMillis()

            val trigger = opMode.gp1.current.rightTrigger - opMode.gp1.current.leftTrigger
            if (trigger > triggerThreshold) lastIntake = now
            if (trigger.absoluteValue < triggerThreshold && now < lastIntake + outtakeBurstDuration) {
                intake.targetPower = outtakeBurstPower
                return@OpModeLoop
            }
            intake.targetPower = trigger
        }
    }

    companion object {
        @JvmField
        var triggerThreshold = 0.2
    }
}