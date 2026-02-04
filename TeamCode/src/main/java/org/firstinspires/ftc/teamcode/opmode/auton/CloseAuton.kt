package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.WaitFor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Configurable
@Autonomous
class CloseAuton : OpMode() {
    override fun run() {
        val autonManager = AutonManager(this, "closeAuton")
        with(autonManager) {
            scheduler.schedule(Sequential("Close Auton") {
                Command("Start") {
                    WaitFor { isStarted }
                }
                +runPath(0)
            })
        }
    }

    companion object
}