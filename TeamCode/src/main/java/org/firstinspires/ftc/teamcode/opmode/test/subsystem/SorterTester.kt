package org.firstinspires.ftc.teamcode.opmode.test.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp
class SorterTester : OpMode() {
    override fun run() {
        val sorter = Sorter(this)

        scheduler.schedule(
            Command("teleop loop")
            {
                OpModeLoop(this@SorterTester) {
                }
            })
    }

    companion object {
    }
}