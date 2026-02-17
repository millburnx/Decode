package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Hood
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class Teleop : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)

        val rapidFire = Command("Rapid Fire") {
            sorter.isFrontUp = true
            SleepFor { upDuration }
            sorter.isFrontUp = false
            SleepFor { downDuration }

            sorter.isSideUp = true
            SleepFor { upDuration }
            sorter.isSideUp = false
            SleepFor { downDuration }

            sorter.isBackUp = true
            SleepFor { upDuration }
            sorter.isBackUp = false
        }

        scheduler.schedule(Command("Rapid Fire Scheduler") {
            OpModeLoop(this@Teleop) {
                if (!gp1.prev.dPad.down && gp1.current.dPad.down) {
                    rapidFire.cancel()
                    scheduler.schedule(rapidFire)
                }
            }
        })

        scheduler.schedule(Command("Power adjusts") {
            OpModeLoop(this@Teleop) {
                hood.target = hoodOverride
                intake.power = intakeOverride
            }
        })
    }

    companion object {
        @JvmField
        var upDuration = 100L

        @JvmField
        var downDuration = 50L

        @JvmField
        var hoodOverride = 0.5

        @JvmField
        var intakeOverride = 0.0
    }
}