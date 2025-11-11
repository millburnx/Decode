package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.roundTo
import org.firstinspires.ftc.teamcode.opmode.OpMode

class AutoTargetting(opMode: OpMode, pedro: Pedro, apriltags: Apriltags) : Subsystem("Auto Targeting") {

    var enabled = false
    var target: Pose2d? = null

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                val apriltag = apriltags.apriltags[20] ?: apriltags.apriltags[24]
                if (apriltag != null && !enabled) target = apriltag.toPose(pedro.pose)

                tel.addData("target x", target?.x?.roundTo(2) ?: -1.0)
                tel.addData("target y", target?.y?.roundTo(2) ?: -1.0)
                tel.addData("target heading", target?.heading?.roundTo(2) ?: -1.0)

                tel.addData("tag range", apriltag?.ftcPose?.range?.roundTo(2) ?: -1.0)
                tel.addData("tag bearing", apriltag?.ftcPose?.bearing?.roundTo(2) ?: -1.0)

                if (gp1.current.y && !gp1.prev.y) {
                    enabled = !enabled
                }
                val newLocked = enabled && target != null
                if (newLocked != pedro.isLocked) {
                    pedro.isLocked = newLocked
                    if (newLocked) {
                        pedro.follower.turnTo(pedro.pose.position.angle(target!!.position))
                    } else {
                        pedro.follower.startTeleopDrive(true)
                    }
                }
                sync()
            }
        }
    }
}