package org.firstinspires.ftc.teamcode

import com.millburnx.cmdx.Command
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel


@TeleOp(name = "FlyWheelTest")
class FlyWheelTest : LinearOpMode() {
    val scheduler = CommandScheduler()

    override fun runOpMode() {
        val hubs = hardwareMap.getAll(LynxModule::class.java) as MutableList<LynxModule>
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO }

        val flyWheel = FlyWheel(this)

        telemetry.isAutoClear = true

        waitForStart()

        scheduler.schedule(flyWheel.command)

        while (opModeIsActive() && !isStopRequested) {
        }
    }
}
