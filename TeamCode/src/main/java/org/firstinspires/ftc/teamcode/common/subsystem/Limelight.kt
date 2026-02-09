package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.util.Pose2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.util.DeltaTime
import org.firstinspires.ftc.teamcode.common.util.LimelightUtil
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import kotlin.math.abs

class Limelight(
    val opMode: OpMode,
    val getPose: () -> Pose2d,
    val setPose: (Pose2d) -> Unit,
    val deltaTime: DeltaTime
) : Subsystem("Limelight") {
    val xKalmanFilter = OdomKalmanFilter(getPose().x, deltaTime)
    val yKalmanFilter = OdomKalmanFilter(getPose().y, deltaTime)

    var localizationState: LocalizationState =
        if (GlobalStore.autonPose == null) LocalizationState.NONE else LocalizationState.READY

    @Suppress("MemberNameEqualsClassName")
    val limelight = (opMode.hardwareMap["limelight"] as Limelight3A).apply {
        pipelineSwitch(0)
        start()
    }

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            updatePose()
            updateShooter()
        }
    }

    fun updateShooter() {
        // update turret, flywheel, hood
    }

    fun updatePose() {
        if (localizationState == LocalizationState.NONE) {
            // if pose was reset, use mt1 to relocalize
            val pose = getLLPose(false) ?: return
            setPose(pose)
            localizationState = LocalizationState.READY
        }
        // if pose was saved, correct errors with kf
        mergePose(getLLPose() ?: return)
    }

    fun getLLPose(useMT2: Boolean = true): Pose2d? {
        val result = limelight.latestResult
        if (result == null || !result.isValid) return null
        return LimelightUtil.mtToPedro(if (useMT2) result.botpose_MT2 else result.botpose)
    }

    fun mergePose(llPose: Pose2d) {
        val kfX = xKalmanFilter.update(llPose.x)
        val kfY = yKalmanFilter.update(llPose.y)
        setPose(
            Pose2d(
                kfX,
                kfY,
                getPose().heading
            )
        )
    }


    enum class LocalizationState {
        NONE,
        READY;
    }
}

class OdomKalmanFilter(
    var estimate: Double,
    val deltaTime: DeltaTime
) {
    private var errorCovariance = 1.0


    fun update(measurement: Double): Double {
        errorCovariance += processNoise * deltaTime.dt

        val kalmanGain =
            errorCovariance / (errorCovariance + measurementNoise)

        val delta = kalmanGain * (measurement - estimate)
        if (abs(delta) > acceptableDeviation) return estimate

        errorCovariance *= (1.0 - kalmanGain)

        return estimate
    }

    companion object {
        var processNoise: Double = 0.2 // q -  pp distrust
        var measurementNoise: Double = 0.5 // r - ll distrust
        var acceptableDeviation: Double = 1.0
    }
}