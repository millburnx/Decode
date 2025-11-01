package org.firstinspires.ftc.teamcode.auton

import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
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
import org.firstinspires.ftc.teamcode.util.toPedro


@TeleOp(name = "Experimental Auton")
class ExperimentalAuton : LinearOpMode() {
    val timer = ElapsedTime()
    val scheduler = CommandScheduler().apply {
        onSync = {
            val loopHertz = 1.0 / timer.seconds()
            timer.reset()

            telemetry.addData("hz", loopHertz)
            telemetry.addData("cmd", this.runner.commandList.joinToString(", "))
            telemetry.update()

            hubs.forEach { it.clearBulkCache() }
            ManualManager.update()
        }
    }

    var hubs: List<LynxModule> = emptyList()

    fun Path1(pedro: PedroDrive, intake: Intake) =
        pedro.follower
            .pathBuilder()
            .addPath(
                BezierLine(Pose2d(56.000, 8.000).toPedro(), Pose(56.000, 12.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(112.0))
            .build();


    fun Path2(pedro: PedroDrive) =
        pedro.follower
            .pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(56.000, 12.000),
                    Pose(56.000, 36.000),
                    Pose(44.000, 36.000)
                )
            )
            .setTangentHeadingInterpolation()
            .build();

    fun Path3(pedro: PedroDrive) =
        pedro.follower
            .pathBuilder()
            .addPath(
                BezierLine(Pose(44.000, 36.000), Pose(12.000, 36.000))
            )
            .setTangentHeadingInterpolation()
            .build();

    fun Path4(pedro: PedroDrive) =
        pedro.follower
            .pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(12.000, 36.000),
                    Pose(12.000, 12.000),
                    Pose(56.000, 12.000)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(112.0))
            .build();

    override fun runOpMode() {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        ManualManager.init()

        val pedro = PedroDrive(this, Pose2d(56.0, 8.0, 90.0))
        val intake = Intake(this)
        val flyWheel = FlyWheel(this)
        val uppies = Uppies(this, intake, flyWheel)

        intake.locked = true // get rid of gamepad control

        val path1 = Path1(pedro, intake)
        val path2 = Path2(pedro)
        val path3 = Path3(pedro)
        val path4 = Path4(pedro)

        telemetry.isAutoClear = true

        uppies.next()
        scheduler.schedule(uppies.command)

        waitForStart()

        scheduler.schedule(pedro.command)
        scheduler.schedule(intake.command)
        scheduler.schedule(flyWheel.command)

        scheduler.schedule(Sequential {
//            Command { flyWheel.running = true }
//            Command { uppies.autoFireCommand }
            +FollowPathCommand(pedro.follower, path1)
            +uppies.autoFireCommand
            +FollowPathCommand(pedro.follower, path2)
            Command { intake.power = 1.0 }
            +FollowPathCommand(pedro.follower, path3)
            +FollowPathCommand(pedro.follower, path4)
            +uppies.autoFireCommand
            // you can also use integrate stuff by manually calling the stuff
            // rather than pedro callbacks. the follow path command is quite simple.
        })

        while (opModeIsActive() && !isStopRequested) {
        }
    }
}
