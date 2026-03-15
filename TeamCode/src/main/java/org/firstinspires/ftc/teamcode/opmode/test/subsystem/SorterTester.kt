package org.firstinspires.ftc.teamcode.opmode.test.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.IndicatorLight
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Disabled
@Configurable
@TeleOp
class SorterTester : OpMode() {
    override fun run() {
        val indicatorLight = IndicatorLight(this)
        val sorter = Sorter(this, indicatorLight)

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