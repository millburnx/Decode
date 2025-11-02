package org.firstinspires.ftc.teamcode

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
import org.firstinspires.ftc.teamcode.util.APIDFController
import org.firstinspires.ftc.teamcode.util.ManualManager
import org.firstinspires.ftc.teamcode.util.Vec2d
import org.firstinspires.ftc.teamcode.util.deg
import org.firstinspires.ftc.teamcode.util.rad
import kotlin.math.absoluteValue


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

        val assistPID = APIDFController(Assist.kP, Assist.kI, Assist.kD, 0.0)

        var offsetHeading = 0.0

        scheduler.schedule(pedro.command)
        scheduler.schedule(Command("pedro teleop") {
            while (opModeIsActive() && !isStopRequested) {
                val override = (gamepad2.left_stick_x.absoluteValue > 0.3 || gamepad2.left_stick_y.absoluteValue > 0.3)

                val rx = if (gamepad2.right_stick_x.absoluteValue > 0.5 || gamepad2.right_stick_y.absoluteValue > 0.5) {
                    assistPID.p = Assist.kP
                    assistPID.i = Assist.kI
                    assistPID.d = Assist.kD

                    val targetHeading = Vec2d().angleTo(Vec2d(gamepad2.right_stick_x, -gamepad2.right_stick_y))
                    val error = assistPID.calculate((pedro.follower.pose.heading - offsetHeading).deg(), targetHeading.deg())
                    telemetry.addLine("$targetHeading $error")
                    error
                } else {
                    -gamepad1.right_stick_x.toDouble()
                }

                telemetry.addData("override", override)

//                pedro.follower.setTeleOpDrive(
//                    (if (override) -gamepad2.left_stick_y else -gamepad1.left_stick_x).toDouble(),
//                    (if (override) -gamepad2.left_stick_x else gamepad1.left_stick_y).toDouble(),
//                    rx,
//                    !override, // Robot Centric
//                    offsetHeading + 90.0.rad(),
//                )
                if (override) {
                    pedro.follower.setTeleOpDrive(
                        -gamepad2.left_stick_y.toDouble(),
                        -gamepad2.left_stick_x.toDouble(),
                        rx,
                        true,
                        offsetHeading + 90.0.rad()
                    )
                } else {
                    pedro.follower.setTeleOpDrive(
                        -gamepad1.left_stick_y.toDouble(),
                        -gamepad1.left_stick_x.toDouble(),
                        rx,
                        true
                    )
                }
                sync()
            }
        })

        scheduler.schedule(intake.command)
        scheduler.schedule(flyWheel.command)
        scheduler.schedule(uppies.command)

        var flyWheelClose = true

        scheduler.schedule(Command {
            while (opModeIsActive()) {
                if (gamepad1.bWasPressed()) {
                    println("${uppies.autoFireCommandTeleop.job} | ${uppies.autoFireCommandTeleop.job?.isActive}")
                    if (uppies.autoFireCommandTeleop.job?.isActive == true) {
                        // TODO: I don't think this canceling works? But you can ignore if its not a quick fix
                        // Cuz theoretically the library should be fine but who knows lol
                        uppies.autoFireCommandTeleop.cancel()
                        println("CANCELING AUTO")
                    } else {
                        scheduler.schedule(uppies.autoFireCommandTeleop)
                    }
                }
                if (gamepad2.xWasPressed()) {
                    offsetHeading = pedro.follower.pose.heading
                }
                if (gamepad2.leftBumperWasPressed()) {
                    if (flyWheelClose) {
                        FlyWheel.TeleopClosePower -= 0.05
                    } else {
                        FlyWheel.FarPower -= 0.05
                    }
                }
                if (gamepad2.rightBumperWasPressed()) {
                    if (flyWheelClose) {
                        FlyWheel.TeleopClosePower += 0.05
                    } else {
                        FlyWheel.FarPower += 0.05
                    }
                }
                flyWheel.power = if (flyWheelClose) FlyWheel.TeleopClosePower else FlyWheel.FarPower
                telemetry.addData("flywheel power - $flyWheelClose", flyWheel.power)
                sync()
            }
        })

        if (gamepad2.dpad_right) flyWheelClose = true
        if (gamepad2.dpad_left) flyWheelClose = false

        while (opModeIsActive() && !isStopRequested) {
        }
        scheduler.runner.cancel()
    }

}

@Configurable
object Assist {
    @JvmField
    var kP = 0.05

    @JvmField
    var kI = 0.0

    @JvmField
    var kD = 0.0
}