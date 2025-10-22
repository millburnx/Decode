package org.firstinspires.ftc.teamcode

import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel
import org.firstinspires.ftc.teamcode.util.ManualManager


@TeleOp(name = "FlyWheelTest")
class FlyWheelTest : LinearOpMode() {

    var hubs: List<LynxModule> = emptyList()

    val timer = ElapsedTime()
    val scheduler = CommandScheduler().apply {
        onSync = {
            val loopHertz = 1.0 / timer.seconds()
            telemetry.addData("hz", loopHertz)
            timer.reset()

            telemetry.update()

            hubs.forEach { it.clearBulkCache() }
            ManualManager.update()
        }
    }

    override fun runOpMode() {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        ManualManager.init()

        val flyWheel = FlyWheel(this)

        telemetry.isAutoClear = true

        waitForStart()

        scheduler.schedule(flyWheel.command)

        while (opModeIsActive() && !isStopRequested) {
        }
    }
}
