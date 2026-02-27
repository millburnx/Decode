package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.PanelsField
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.paths.PedroBuilder
import com.millburnx.cmdxpedro.paths.heading.ConstantHeading
import com.millburnx.cmdxpedro.paths.heading.LinearHeading
import com.millburnx.cmdxpedro.paths.heading.TangentialHeading
import com.millburnx.cmdxpedro.paths.path.CubicBezier
import com.millburnx.cmdxpedro.paths.path.Line
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.Pose2d
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.subsystem.*
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Configurable
@Autonomous
class CloseAuton : OpMode() {
    override fun run() {
        val sorter = Sorter(this)
        val hood = Hood(this)
        val flyWheel = FlyWheel(this)
        val intake = Intake(this)
        val drive = Drive(this)
        val turret = Turret(this, { drive.pose.heading }, { drive.velocity.heading }, { voltageSensor.voltage })
        val autoAdjust = AutoAdjust(this, flyWheel, hood, { drive.pose })

        turret.targetingMode = Turret.TargetingMode.GLOBAL
        turret.target = 45.0

        val fire = Command {
            flyWheel.state = FlyWheel.FlyWheelState.SHOOTING

            WaitFor { turret.atTarget && turret.isSteady }

            val minRPM = { autoAdjust.minRapidRPM - FlyWheel.Controller.rpmThreshold }
            val maxRPM = { flyWheel.shootingRPM + FlyWheel.Controller.rpmThreshold }

            val atRPM = { flyWheel.rpm in minRPM()..maxRPM() }

            WaitFor { atRPM() }
            sorter.frontPod.isUp = true
            SleepFor { upDuration }
            sorter.frontPod.isUp = false
            SleepFor { downDuration }

            WaitFor { atRPM() }
            sorter.sidePod.isUp = true
            SleepFor { upDuration }
            sorter.sidePod.isUp = false
            SleepFor { downDuration }

            WaitFor { atRPM() }
            sorter.backPod.isUp = true
            SleepFor { upDuration }
            sorter.backPod.isUp = false
            SleepFor { downDuration }

            flyWheel.state = FlyWheel.FlyWheelState.IDLE
        }

        scheduler.schedule(Command {
            SleepFor { 1000 }
            drive.pose = Pose2d(111.5, 137.0, -90.0)
        })

        val builder = PedroBuilder(false)

        scheduler.schedule(Command("general") {
            val canvas = PanelsField.field
            canvas.setOffsets(PanelsField.presets.PEDRO_PATHING)
            while (!isStopRequested) {
                val pose = drive.pose
                tel.addData("pose.x", pose.x)
                tel.addData("pose.y", pose.y)
                tel.addData("pose.h", pose.heading)
                canvas.moveCursor(pose.x, pose.y)
                canvas.setStyle(fill = "none", outline = "white", width = 1.5)
                canvas.circle(8.0)
                val lookPos = pose + Vec2d(1.0, 0.0).rotate(pose.radians) * 8.0
                canvas.line(lookPos.x, lookPos.y)
                sync()
            }
        })

        scheduler.schedule(Sequential("Close Auton") {
            Command("Start") {
                WaitFor { isStarted }
            }
            +builder.PathCommand(
                drive.follower,
                Line(
                    Vec2d(111.5, 137.0),
                    Vec2d(96.0, 96.0),
                ),
                LinearHeading(-90.0, -0.0),
                { !isStopRequested }
            )
            Command { SleepFor { stablizationTime } }
            +fire
            Command { intake.power = 1.0 }
            +builder.PathCommand(
                drive.follower,
                Line(
                    Vec2d(96.0, 96.0),
                    Vec2d(120.0, 96.0),
                ),
                ConstantHeading(-0.0),
                { !isStopRequested },
            )
            +builder.PathCommand(
                drive.follower,
                Line(
                    Vec2d(120.0, 96.0),
                    Vec2d(96.0, 96.0),
                ),
                LinearHeading(0.0, -90.0),
                { !isStopRequested }
            )
            Command { SleepFor { stablizationTime } }
            +fire
            +builder.PathCommand(
                drive.follower,
                CubicBezier(
                    Vec2d(96.0, 96.0),
                    Vec2d(120.0, 96.0),
                    Vec2d(120.0, 96.0),
                    Vec2d(120.0, 60.0),
                ),
                ConstantHeading(-90.0),
                { !isStopRequested }
            )
            +builder.PathCommand(
                drive.follower,
                Line(
                    Vec2d(120.0, 60.0),
                    Vec2d(96.0, 96.0),
                ),
                ConstantHeading(-90.0),
                { !isStopRequested }
            )
            Command { SleepFor { stablizationTime } }
            +fire
            +builder.PathCommand(
                drive.follower,
                CubicBezier(
                    Vec2d(96.0, 96.0),
                    Vec2d(120.0, 96.0),
                    Vec2d(120.0, 96.0),
                    Vec2d(120.0, 30.0),
                ),
                ConstantHeading(-90.0),
                { !isStopRequested }
            )
            +builder.PathCommand(
                drive.follower,
                Line(
                    Vec2d(120.0, 30.0),
                    Vec2d(96.0, 96.0),
                ),
                TangentialHeading(false),
                { !isStopRequested }
            )
            Command { SleepFor { stablizationTime } }
            +fire
        })
    }

    companion object {
        @JvmField
        var upDuration = 250L

        @JvmField
        var downDuration = 50L

        @JvmField
        var stablizationTime = 500L
    }
}