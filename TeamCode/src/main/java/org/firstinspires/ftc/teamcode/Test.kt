package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel
import org.firstinspires.ftc.teamcode.subsystems.Uppies


@TeleOp(name = "Test")
class Test : LinearOpMode() {
    val scheduler = CommandScheduler()
    val tel = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        val hubs = hardwareMap.getAll(LynxModule::class.java) as MutableList<LynxModule>
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO }

        val flyWheel = FlyWheel(this)
        val uppies = Uppies(this, tel)

//        super.telemetry.isAutoClear = true

        waitForStart()

        scheduler.schedule(flyWheel.command)
        scheduler.schedule(uppies.command)

        while (opModeIsActive() && !isStopRequested) {
        }
    }
}
