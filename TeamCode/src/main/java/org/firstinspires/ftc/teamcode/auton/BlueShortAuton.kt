package org.firstinspires.ftc.teamcode.auton

import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive
import org.firstinspires.ftc.teamcode.subsystems.Uppies
import org.firstinspires.ftc.teamcode.util.ManualManager
import org.firstinspires.ftc.teamcode.util.Pose2d
import org.firstinspires.ftc.teamcode.util.SleepFor


@Autonomous(name = "Blue Short Auton", preselectTeleOp = "Teleop")
class BlueShortAuton : LinearOpMode() {
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
                BezierLine(Pose(28.000, 133.000), Pose(50.000, 110.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(145.0), Math.toRadians(145.0))
            .build();


    fun Path2(pedro: PedroDrive, intake: Intake) =
        pedro.follower
            .pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(50.000, 110.000),
                    Pose(60.000, 98.000),
                    Pose(45.000, 92.000)
                )
            )
            .addParametricCallback(0.5) {
                intake.locked = true; intake.power = -1.0
            }
            .setLinearHeadingInterpolation(Math.toRadians(135.0), Math.toRadians(0.0))
            .build();

    fun Path3(pedro: PedroDrive, uppies: Uppies) =
        pedro.follower
            .pathBuilder()
            .addPath(
                BezierLine(Pose(45.000, 92.000), Pose(20.000, 92.000))
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .addParametricCallback(1.0) {
                uppies.next()
            }
            .build();

    fun Path4(pedro: PedroDrive) =
        pedro.follower
            .pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(20.000, 92.000),
                    Pose(40.000, 92.000),
                    Pose(50.000, 110.000)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(135.0))
            .build();

    fun Path5(pedro: PedroDrive) =
        pedro.follower
            .pathBuilder()
            .addPath(
                BezierLine(Pose(40.000, 110.000), Pose(60.000, 130.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(135.0), Math.toRadians(90.0))
            .build();

    override fun runOpMode() {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        ManualManager.init()

        val pedro = PedroDrive(this, Pose2d(28.0, 133.0, 145.0))
        val intake = Intake(this)
        val flyWheel = FlyWheel(this)
        val uppies = Uppies(this, intake, flyWheel)

        intake.locked = true // get rid of gamepad control

        val path1 = Path1(pedro, intake)
        val path2 = Path2(pedro, intake)
        val path3 = Path3(pedro,uppies)
        val path4 = Path4(pedro)
        val path5 = Path5(pedro)

        telemetry.isAutoClear = true

        uppies.next()
        scheduler.schedule(uppies.command)

        waitForStart()

        scheduler.schedule(pedro.command)
        scheduler.schedule(intake.command)
        scheduler.schedule(flyWheel.command)
        flyWheel.power = FlyWheel.ClosePower

        scheduler.schedule(Sequential {
//            Command { flyWheel.running = true }
//            Command { uppies.autoFireCommand }
            +FollowPathCommand(pedro.follower, path1)
            +uppies.autoFireCommand
            +FollowPathCommand(pedro.follower, path2)
//            Command { intake.locked = true; intake.power = -1.0 }
            +FollowPathCommand(pedro.follower, path3)
            Command { SleepFor (1000) }
            +FollowPathCommand(pedro.follower, path4)
            +uppies.autoFireCommand
            +FollowPathCommand(pedro.follower, path5)
        })

        while (opModeIsActive() && !isStopRequested) {
        }
        scheduler.runner.cancel()
    }
}
