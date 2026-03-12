package org.firstinspires.ftc.teamcode.opmode.test.pedro

import com.bylazar.configurables.PanelsConfigurables
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.paths.heading.LinearHeading
import com.millburnx.cmdxpedro.paths.path.Line
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.TeleOpDrive
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.opmode.auton.BaseAutonManager
import org.firstinspires.ftc.teamcode.pedro.Tuning

@Configurable
@TeleOp
class LineTest : OpMode() {
    override fun run() {
        PanelsConfigurables.refreshClass(Tuning::class.java)

        val drive = TeleOpDrive(this, true)

        val autonManager = BaseAutonManager(this, drive, false)

        val forward = {
            autonManager.builder.PathCommand(
                drive.follower, Line(
                    Vec2d(0.0, 0.0),
                    Vec2d(distance, 0.0),
                ),
                LinearHeading(0.0, 0.0),
                { !isStopRequested }
            )
        }
        val backward = {
            autonManager.builder.PathCommand(
                drive.follower, Line(
                    Vec2d(distance, 0.0),
                    Vec2d(0.0, 0.0),
                ),
                LinearHeading(0.0, 0.0),
                { !isStopRequested }
            )
        }

        var isBusy = false

        scheduler.schedule(
            Command {
                while (!isStopRequested) {
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
        var distance = 48.0

        @JvmField
        var isRunning = false
    }
}