package org.firstinspires.ftc.teamcode.common.commands

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.millburnx.util.toRadians
import com.pedropathing.geometry.BezierLine
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.Path
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.common.subsystem.TeleOpDrive

@Suppress("FunctionName")
fun AutoPark(drive: TeleOpDrive, isRed: Boolean, isRunning: () -> Boolean): Command {
    return Command("Auto Park", {
        drive.isTeleopDrive = true
    })
    {
        drive.isTeleopDrive = false
        drive.follower.followPath(
            drive.follower.pathBuilder()
                .addPath(
                    Path(
                        BezierLine(
                            { drive.pose.toPedro() },
                            Pose2d(105.0, 33.0, 0.0).mirror(!isRed).toPedro()
                        )
                    )
                )
                .setHeadingInterpolation(
                    HeadingInterpolator.linearFromPoint(
                        { drive.pose.radians },
                        (0.0).toRadians(),
                        0.5
                    )
                )
                .build()
        )
        WaitFor { !isRunning() || drive.follower.atParametricEnd() }
        drive.follower.breakFollowing()
        drive.isTeleopDrive = true
    }
}