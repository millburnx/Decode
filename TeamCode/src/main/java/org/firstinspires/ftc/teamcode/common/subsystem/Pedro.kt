package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.toDegrees
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.abs
import kotlin.math.pow

@Configurable
class Pedro(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Pedro") {
    val follower = Constants.createFollower(opMode.hardwareMap).apply {
        setStartingPose(Pose(0.0, 0.0, 0.0))
    }
    var isLocked = false;

    val pose: Pose2d
        get() = Pose2d(follower.pose.x, follower.pose.y, follower.pose.heading.toDegrees())

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            follower.update()
            if (isTeleop) follower.startTeleopDrive(true)
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                follower.update()
                if (isTeleop && !isLocked) {
                    // axis snapping
                    val snappedX = snapTo(gp1.current.leftJoyStick.x, 0.0, snapThreshold)
                    val snappedY = snapTo(gp1.current.leftJoyStick.y, 0.0, snapThreshold)

                    follower.setTeleOpDrive(
                        -snappedX.pow(3),
                        -snappedY.pow(3),
                        -gp1.current.rightJoyStick.x.pow(3)
                    )
                }
                sync()
            }
        }
    }

    private fun snapTo(raw: Double, target: Double, threshold: Double): Double {
        if (abs(raw - target) < threshold) return target
        return raw
    }

    companion object {
        @JvmField
        var snapThreshold = 0.1;
    }
}