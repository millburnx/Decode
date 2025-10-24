package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Uppies
import org.firstinspires.ftc.teamcode.util.ManualManager
import org.firstinspires.ftc.teamcode.util.SleepFor


@TeleOp(name = "IntakeDrive")
class IntakeDrive : LinearOpMode() {
    val timer = ElapsedTime()
    val scheduler = CommandScheduler().apply {
        onSync = {
            val loopHertz = 1.0 / timer.seconds()
            tel.addData("hz", loopHertz)
            timer.reset()

            tel.update()

            hubs.forEach { it.clearBulkCache() }
            ManualManager.update()
        }
    }

    val tel = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    var hubs: List<LynxModule> = emptyList()

    override fun runOpMode() {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        ManualManager.init()

        val drivetrain = Drivetrain(this)
        val intake = Intake(this)
        val flyWheel = FlyWheel(this)
        val uppies = Uppies(this, tel, intake, flyWheel)

        telemetry.isAutoClear = true

        waitForStart()

        scheduler.schedule(drivetrain.command)
        scheduler.schedule(intake.command)
        scheduler.schedule(flyWheel.command)
        scheduler.schedule(uppies.command)

        var prevAutoFireButton = gamepad1.b
        while (opModeIsActive() && !isStopRequested) {
            val newAutoFireButton = gamepad1.b
            if (newAutoFireButton && !prevAutoFireButton) {
                println("${uppies.autoFireCommand.job} | ${uppies.autoFireCommand.job?.isActive}")
                if (uppies.autoFireCommand.job?.isActive == true) {
                    uppies.autoFireCommand.cancel()
                    println("CANCELING AUTO")
                    scheduler.schedule(Command("Auto Canceller") {
                        SleepFor(1000)
                        intake.power = 0.0
                        intake.locked = false
                        flyWheel.running = false
                        println("CANCELING AUTO 2")
                    })
                } else {
                    scheduler.schedule(uppies.autoFireCommand)
                }
            }
            prevAutoFireButton= newAutoFireButton
        }
    }
}
