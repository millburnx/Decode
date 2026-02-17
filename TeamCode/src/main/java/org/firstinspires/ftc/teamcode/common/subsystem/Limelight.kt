package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.util.Pose2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.hardware.fromFTC
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
                    val pose = Pose2d.fromFTC(result.botpose)
                    this@Limelight.pose = pose to System.nanoTime()
                    setPose(pose)
                    localizationState = LocalizationState.READY
                }
            }
            if (localizationState == LocalizationState.READY) {
                limelight.updateRobotOrientation(getPose().heading)
                val result = limelight.getLatestResult()
                if (result != null && result.isValid) {
                    pose = Pose2d.fromFTC(result.botpose_MT2) to System.nanoTime()
                }
            }
        }
    }

    enum class LocalizationState {
        NONE,
        READY;
    }
}