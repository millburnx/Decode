package org.firstinspires.ftc.teamcode

import com.arcrobotics.ftclib.controller.PIDController
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive
import org.firstinspires.ftc.teamcode.subsystems.Uppies
import org.firstinspires.ftc.teamcode.util.ManualManager
import org.firstinspires.ftc.teamcode.util.Vec2d
import org.firstinspires.ftc.teamcode.util.deg


@TeleOp(name = "Teleop")
class Teleop : LinearOpMode() {
    val timer = ElapsedTime()
    val scheduler = CommandScheduler().apply {
        onSync = {
            val loopHertz = 1.0 / timer.seconds()
            timer.reset()

            telemetry.addData("hz", loopHertz)
            telemetry.update()

            hubs.forEach { it.clearBulkCache() }
            ManualManager.update()
        }
    }

    var hubs: List<LynxModule> = emptyList()

    override fun runOpMode() {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        ManualManager.init()

        val pedro = PedroDrive(this)
        val intake = Intake(this)
        val flyWheel = FlyWheel(this)
        val uppies = Uppies(this, intake, flyWheel)

        telemetry.isAutoClear = true

        waitForStart()

        pedro.follower.startTeleopDrive();

        var headingLock = false
        val assistPID = PIDController(Assist.kP, Assist.kI, Assist.kD)

        scheduler.schedule(pedro.command)
        scheduler.schedule(Command("pedro teleop") {
                while (opModeIsActive() && !isStopRequested) {
                    if (!headingLock) {
                        pedro.follower.setTeleOpDrive(
                            -gamepad1.left_stick_y.toDouble(),
                            -gamepad1.left_stick_x.toDouble(),
                            -gamepad1.right_stick_x.toDouble(),
                            true // Robot Centric
                        )
                    } else {
                        assistPID.p = Assist.kP
                        assistPID.i = Assist.kI
                        assistPID.d = Assist.kD

                        val targetHeading = Vec2d().angleTo(Vec2d(gamepad2.right_stick_x, -gamepad2.right_stick_y))
                        val error = assistPID.calculate(pedro.follower.pose.heading.deg(), targetHeading.deg())

                        pedro.follower.setTeleOpDrive(
                            -gamepad1.left_stick_y.toDouble(),
                            -gamepad1.left_stick_x.toDouble(),
                            error,
                            true // Robot Centric
                        )
                    }
                    sync()
            }
        })
        scheduler.schedule(intake.command)
        scheduler.schedule(flyWheel.command)
        scheduler.schedule(uppies.command)

        var prevAutoFireButton = gamepad1.b
        var prevHeadingLock = gamepad1.dpad_right
        while (opModeIsActive() && !isStopRequested) {
            val newAutoFireButton = gamepad1.b
            if (newAutoFireButton && !prevAutoFireButton) {
                println("${uppies.autoFireCommand.job} | ${uppies.autoFireCommand.job?.isActive}")
                if (uppies.autoFireCommand.job?.isActive == true) {
                    // TODO: I don't think this canceling works? But you can ignore if its not a quick fix
                    // Cuz theoretically the library should be fine but who knows lol
                    uppies.autoFireCommand.cancel()
                    println("CANCELING AUTO")
                } else {
                    scheduler.schedule(uppies.autoFireCommand)
                }
            }
            prevAutoFireButton = newAutoFireButton

            val newHeadingLock = gamepad1.dpad_left
            if (newHeadingLock && !prevHeadingLock) {
                headingLock = !headingLock
            }
            telemetry.addData("headingLock", headingLock)
            prevHeadingLock = newHeadingLock
        }
        scheduler.runner.cancel()
    }

}

@Configurable
object Assist {
    @JvmField
    var kP = 0.0
    @JvmField
    var kI = 0.0
    @JvmField
    var kD = 0.0
}