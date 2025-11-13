package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d
import com.millburnx.cmdxpedro.util.toRadians
import org.firstinspires.ftc.teamcode.common.vision.VisionManager
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@Configurable
class Apriltags : Subsystem("Apriltags") {
    val processor: AprilTagProcessor = AprilTagProcessor.Builder()
        .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .build().apply {
            setDecimation(decimation)
//            setPoseSolver(PoseSolver.OPENCV_IPPE)
        }

    init {
        VisionManager.processors.add(processor)
    }

    val apriltags: Map<Int, AprilTagDetection>
        get() = processor.detections.associateBy { it.id }

    override val run: suspend Command.() -> Unit = {
        // we don't actually have to do anything here
        // vision portal handles running the pipeline
        // and calculations are handled by the user
    }

    companion object {
        @JvmField
        var decimation = 2F
    }
}

fun AprilTagDetection.toPose(currentPose: Pose2d): Pose2d {
    val offset = Vec2d(ftcPose.range).rotate(currentPose.radians + ftcPose.bearing.toRadians())
    val apriltagPose = Pose2d(currentPose.position + offset, currentPose.heading + ftcPose.yaw)
    val tagOffset = Vec2d(AutoTargeting.goalDepth).rotate(currentPose.radians + ftcPose.yaw.toRadians())
    return apriltagPose + tagOffset
}