package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.util.Pose2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.util.DeltaTime
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class Limelight(
    val opMode: OpMode,
    val getPose: () -> Pose2d,
    val setPose: (Pose2d) -> Unit,
    val deltaTime: DeltaTime
) : Subsystem("Limelight") {

    var localizationState: LocalizationState =
        if (GlobalStore.autonPose == null) LocalizationState.NONE else LocalizationState.READY

    @Suppress("MemberNameEqualsClassName")
    val limelight = (opMode.hardwareMap["limelight"] as Limelight3A).apply {
        pipelineSwitch(0)
        start()
    }

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            updateShooter()
        }
    }

    fun updateShooter() {
        // update turret, flywheel, hood
    }

    enum class LocalizationState {
        NONE,
        READY;
    }
}