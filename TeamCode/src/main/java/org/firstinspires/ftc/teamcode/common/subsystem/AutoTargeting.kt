package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.roundTo
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.pow

@Configurable
class AutoTargeting(opMode: OpMode, pedro: Pedro, apriltags: Apriltags, setFlyWheelVelocity: (Double) -> Unit = {}) :
    Subsystem("Auto Targeting") {

    var enabled = false
    var target: Pose2d? = null
    var targetATag: AprilTagDetection? = null

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                val apriltag = apriltags.apriltags[Apriltags.BLUE_ID] ?: apriltags.apriltags[Apriltags.RED_ID]
                if (apriltag != null && !enabled) {
                    targetATag = apriltag
                    target = apriltag.toPose(pedro.pose)
                }

                tel.addData("target x", target?.x?.roundTo(2) ?: -1.0)
                tel.addData("target y", target?.y?.roundTo(2) ?: -1.0)
                tel.addData("target heading", target?.heading?.roundTo(2) ?: -1.0)

                if (target != null) {
                    val distance = pedro.pose.distanceTo(target!!)
                    tel.addData("distance to target", distance)
                    setFlyWheelVelocity(getFlyWheelPower(distance))
                } else {
                    tel.addData("distance to target", -1.0)
                    setFlyWheelVelocity(FlyWheel.ShootingVelocity)
                }

                tel.addData("tag range", apriltag?.ftcPose?.range?.roundTo(2) ?: -1.0)
                tel.addData("tag bearing", apriltag?.ftcPose?.bearing?.roundTo(2) ?: -1.0)
                tel.addData("tag yaw", apriltag?.ftcPose?.yaw?.roundTo(2) ?: -1.0)

                if (gp1.current.y && !gp1.prev.y) {
                    enabled = !enabled
                }
                val newLocked = enabled && target != null
                if (newLocked != pedro.isLocked) {
                    pedro.isLocked = newLocked
                    if (newLocked) {
                        pedro.follower.turnTo(pedro.pose.angleTo(target!!))
                    } else {
                        pedro.follower.startTeleopDrive(false)
                    }
                }
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var goalDepth = 8.0

        fun getFlyWheelPower(distance: Double): Double {
            return (0.00000530928 * distance.pow(4)
                    - 0.00314799 * distance.pow(3)
                    + 0.685274 * distance.pow(2)
                    - 56.30037 * distance
                    + 3233.6749)
        }
    }
}