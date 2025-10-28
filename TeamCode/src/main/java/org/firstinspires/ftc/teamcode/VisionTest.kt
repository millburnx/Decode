package org.firstinspires.ftc.teamcode

import com.arcrobotics.ftclib.controller.PIDController
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Odom
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.util.ManualManager
import org.firstinspires.ftc.teamcode.util.Pose2d
import org.firstinspires.ftc.teamcode.util.normalizeDegrees
import kotlin.math.abs


@Configurable
@TeleOp(name = "VisionTest")
class VisionTest : LinearOpMode() {
    val timer = ElapsedTime()
    val scheduler = CommandScheduler().apply {
        onSync = {
            val loopHertz = 1.0 / timer.seconds()
            timer.reset()

            telemetry.addData("hz", loopHertz)
//            tel.update()

            hubs.forEach { it.clearBulkCache() }
            ManualManager.update()
        }
    }

//    val tel = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    var hubs: List<LynxModule> = emptyList()

    override fun runOpMode() {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        ManualManager.init()
//        telemetry.isAutoClear = true

        val vision = Vision(this)
        val odom = Odom(this)
        val drivetrain = Drivetrain(this)

        waitForStart()

        scheduler.schedule(vision.command)

        var prevAlignButton = gamepad1.b
        while (opModeIsActive() && !isStopRequested) {
            val newAlignButton = gamepad1.b
            if (newAlignButton && !prevAlignButton) {
                scheduler.schedule(Command("Auto Align") {
                    // TODO: Get this command working, didn't have time to properly test yesterday
                    // Possibly could have errors in the vision subsystem btw
                    val targetPose = vision.targetPose ?: return@Command
                    val anglePid = PIDController(kP, kI, kD)
                    val angleTo = { odom.pose.position.angleTo(targetPose.position) }
                    val getError = {
                        normalizeDegrees(odom.pose.heading - angleTo())
                    }
                    while (abs(getError()) < angleThreshold) {
                        val power = anglePid.calculate(odom.pose.heading, angleTo())
                        drivetrain.drive(Pose2d(0.0, 0.0, power))
                    }
                })
            }
            prevAlignButton = newAlignButton
        }
    }

    companion object {
        @JvmField
        var angleThreshold = 10.0

        @JvmField
        var kP = 0.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0
    }
}
