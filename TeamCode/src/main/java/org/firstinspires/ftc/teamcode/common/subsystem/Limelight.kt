package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.util.Pose2d
import com.millburnx.util.vector.Vec2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.hardware.fromFTC
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

/**
 * Make sure to set getPose and setPose after you construct the localizer
 */
@Configurable
class Limelight(
    val opMode: OpMode,
) : Subsystem("Limelight") {
    var pose: Pair<Pose2d, Long>? = null

    var getPose: () -> Pose2d = { Pose2d() }
    var setPose: (Pose2d) -> Unit = {}
    var drawPose: (Pose2d) -> Unit = {}

    /**
     * Heading of turret in global space
     */
    var turretHeading: () -> Double = { 0.0 }

    var localizationState: LocalizationState =
        if (GlobalStore.autonPose == null) LocalizationState.NONE else LocalizationState.READY

    @Suppress("MemberNameEqualsClassName")
    val limelight = (opMode.hardwareMap["limelight"] as Limelight3A).apply {
        pipelineSwitch(0)
        start()
    }

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            if (localizationState == LocalizationState.NONE) {
                val result = limelight.getLatestResult()
                if (result != null && result.isValid) {
                    val pose = convertLL(Pose2d.fromFTC(result.botpose), useDriveHeading = false)
                    this@Limelight.pose = pose to System.nanoTime()
                    setPose(pose)
                    drawPose(pose)
                    localizationState = LocalizationState.READY
                }
            }
            if (localizationState == LocalizationState.READY) {
                limelight.updateRobotOrientation(normalizeDegrees(getPose().heading + turretHeading()))
                val result = limelight.getLatestResult()
                if (result != null && result.isValid) {
                    val pose = convertLL(Pose2d.fromFTC(result.botpose_MT2))
                    this@Limelight.pose = pose to System.nanoTime()
                    drawPose(pose)
                }
            }
        }
    }

    fun convertLL(pose: Pose2d, useDriveHeading: Boolean = true): Pose2d {
        // the primary thing we need to handle here
        // is conversion from the turret frame to the robot frame
        // so account for the offset of the centers and the heading of the turret

        val newHeading = normalizeDegrees(pose.heading - turretHeading())

        val offset = Vec2d(16.0, -42.0) / 25.4

        val driveHeading = if (useDriveHeading) {
            getPose().heading
        } else {
            newHeading
        }

        val newPos = pose.position - offset.rotate(driveHeading)

        return Pose2d(newPos, newHeading)
    }

    enum class LocalizationState {
        NONE,
        READY;
    }
}