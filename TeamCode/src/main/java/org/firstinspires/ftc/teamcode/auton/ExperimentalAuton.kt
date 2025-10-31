package org.firstinspires.ftc.teamcode.auton

import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.pedropathing.geometry.BezierLine
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive
import org.firstinspires.ftc.teamcode.subsystems.Uppies
import org.firstinspires.ftc.teamcode.util.ManualManager
import org.firstinspires.ftc.teamcode.util.Pose2d
import org.firstinspires.ftc.teamcode.util.rad
import org.firstinspires.ftc.teamcode.util.toPedro

@TeleOp(name = "Experimental Auton")
class ExperimentalAuton : LinearOpMode() {
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

    fun Path1(pedro: PedroDrive, intake: Intake) =
        pedro.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Pose2d(104.0, 9.0).toPedro(),
                    Pose2d(105.0, 106.0).toPedro()
                )
            )
            .setLinearHeadingInterpolation(90.0.rad(), 45.0.rad())
            .addTemporalCallback(500.0) {
                intake.power = 1.0
            }
            .build()

    fun Path2(pedro: PedroDrive) =
        pedro.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Pose2d(105.0, 106.0).toPedro(),
                    Pose2d(124.0, 106.0).toPedro()
                )
            )
            .setTangentHeadingInterpolation()
            .build()


    override fun runOpMode() {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        ManualManager.init()

        val pedro = PedroDrive(this)
        val intake = Intake(this)
        val flyWheel = FlyWheel(this)
        val uppies = Uppies(this, intake, flyWheel)

        intake.locked = true // get rid of gamepad control

        val path1 = Path1(pedro, intake)
        val path2 = Path2(pedro)

        telemetry.isAutoClear = true

        waitForStart()

        scheduler.schedule(pedro.command)
        scheduler.schedule(intake.command)
        scheduler.schedule(flyWheel.command)
        scheduler.schedule(uppies.command)

        scheduler.schedule(Sequential {
            +FollowPathCommand(pedro.follower, path1)
            +FollowPathCommand(pedro.follower, path2)
            // you can also use integrate stuff by manually calling the stuff
            // rather than pedro callbacks. the follow path command is quite simple.
        })

        while (opModeIsActive() && !isStopRequested) {
        }
    }
}
