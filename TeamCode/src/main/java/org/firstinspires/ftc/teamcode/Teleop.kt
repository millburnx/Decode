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
import org.firstinspires.ftc.teamcode.util.RisingEdgeDetector
import org.firstinspires.ftc.teamcode.util.Vec2d
import org.firstinspires.ftc.teamcode.util.deg
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
                    val error = assistPID.calculate(pedro.follower.pose.heading.deg(), targetHeading.deg())
                    error
                } else {
                    -gamepad1.right_stick_x.toDouble()
                }

                pedro.follower.setTeleOpDrive(
                    -(if (override) gamepad2 else gamepad1).left_stick_y.toDouble(),
                    -(if (override) gamepad2 else gamepad1).left_stick_x.toDouble(),
                    rx,
                    override, // Robot Centric
                    offsetHeading,
                )
                sync()
            }
        })

        scheduler.schedule(intake.command)
        scheduler.schedule(flyWheel.command)
        scheduler.schedule(uppies.command)

        val autoFireTrigger = RisingEdgeDetector({ gamepad1.b }) {
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

        scheduler.schedule(autoFireTrigger.command)

        val setHeading = RisingEdgeDetector({ gamepad2.x }) {
            offsetHeading = pedro.follower.pose.heading
        }
        scheduler.schedule(setHeading.command)

        var flyWheelClose = true

        if (gamepad2.dpad_right) flyWheelClose = true
        if (gamepad2.dpad_left) flyWheelClose = false

        val decreasePower = RisingEdgeDetector({ gamepad2.left_bumper }) {
            if (flyWheelClose) {
                FlyWheel.ClosePower -= 0.05
            } else {
                FlyWheel.FarPower -= 0.05
            }
        }
        val increasePower = RisingEdgeDetector({ gamepad2.right_bumper }) {
            if (flyWheelClose) {
                FlyWheel.ClosePower += 0.05
            } else {
                FlyWheel.FarPower += 0.05
            }
        }

        scheduler.schedule(decreasePower.command)
        scheduler.schedule(increasePower.command)
        scheduler.schedule(Command("flywheel power") {
            while (opModeIsActive() && !isStopRequested) {
                flyWheel.power = if (flyWheelClose) FlyWheel.ClosePower else FlyWheel.FarPower
                telemetry.addData("flywheel power - $flyWheelClose", flyWheel.power)
            }
        })

        while (opModeIsActive() && !isStopRequested) {
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