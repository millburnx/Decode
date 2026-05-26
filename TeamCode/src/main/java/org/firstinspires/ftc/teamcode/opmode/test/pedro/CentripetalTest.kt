package org.firstinspires.ftc.teamcode.opmode.test.pedro

import com.bylazar.configurables.PanelsConfigurables
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.paths.heading.TangentialHeading
import com.millburnx.cmdxpedro.paths.path.Path
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.cmdxpedro.util.toPedro
import com.millburnx.util.vector.Vec2d
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.paths.PathBuilder
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.subsystem.teleop.TeleOpDrive
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.opmode.auton.BaseAutonManager
import org.firstinspires.ftc.teamcode.pedro.Tuning

@Disabled
@Configurable
@TeleOp
class CentripetalTest : OpMode() {
    override fun run() {
        PanelsConfigurables.refreshClass(Tuning::class.java)

        val drive = TeleOpDrive(this, true)

        val autonManager = BaseAutonManager(this, drive, false)

        val forward = {
            autonManager.builder.PathCommand(
                drive.follower, NthBezier(
                    listOf(
                        Vec2d(0.0, 0.0),
                        Vec2d(distance, 0.0),
                        Vec2d(distance, distance),
                    )
                ),
                TangentialHeading(false),
                { !isStopRequested }
            )
        }
        val backward = {
            autonManager.builder.PathCommand(
                drive.follower, NthBezier(
                    listOf(
                        Vec2d(distance, distance),
                        Vec2d(distance, 0.0),
                        Vec2d(0.0, 0.0),
                    )
                ),
                TangentialHeading(true),
                { !isStopRequested }
            )
        }

        var isBusy = false

        scheduler.schedule(
            Command {
                WaitFor { isStarted }
                val timer = ElapsedTime()
                while (!isStopRequested) {
                    WaitFor { timer.milliseconds() >= 1000.0 / targetLoopHz || isStopRequested }
                    timer.reset()
                    if (!gp1.prev.rightBumper && gp1.current.rightBumper) {
                        isRunning = !isRunning
                    }
                    if (!isBusy && isRunning) {
                        isBusy = true
                        scheduler.schedule(Sequential {
                            +forward()
                            +backward()
                            Command { isBusy = false }
                        })
                    }
                    sync()
                }
            }
        )
    }

    companion object {
        @JvmField
        var distance = 24.0

        @JvmField
        var isRunning = false

        @JvmField
        var targetLoopHz = 60.0
    }
}

private data class NthBezier(
    val points: List<Vec2d>
) : Path {
    override fun register(pathBuilder: PathBuilder, mirrored: Boolean): PathBuilder {
        return pathBuilder.apply {
            addPath(
                BezierCurve(
                    *points.map { it.mirror(mirrored).toPedro() }.toTypedArray()
                )
            )
        }
    }
}