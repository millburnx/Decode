package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.opmode.OpMode

class AutoTargetting(opMode: OpMode, pedro: Pedro, apriltags: Apriltags) : Subsystem("Auto Targeting") {
    override val run: suspend Command.() -> Unit = {
        with (opMode) {
            WaitFor { isStarted || isStopRequested }
            var enabled = false
            var target: Pose2d? = null
            var prevButton = gp1.y
            while (!isStopRequested) {
                val apriltag = apriltags.apriltags[20] ?: apriltags.apriltags[24]
                if (apriltag != null) target = apriltag.toPose(pedro.pose)

                val currentButton = gp1.y
                if (currentButton && !prevButton) {
                    enabled = !enabled
                }
                prevButton = currentButton

                if (!enabled || target == null) {
                    pedro.isTeleop = true
                    if (!pedro.follower.isTeleopDrive) pedro.follower.startTeleopDrive(true)
                    continue
                }
                pedro.isTeleop = false
                pedro.follower.turnTo(pedro.pose.position.angle(target.position))
            }
        }
    }
}