package org.firstinspires.ftc.teamcode.subsystems

import android.util.Size
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.util.Pose2d
import org.firstinspires.ftc.teamcode.util.Vec2d
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@Configurable
class Vision(opMode: LinearOpMode) : Subsystem("Vision") {
    val aprilTagProcessor = AprilTagProcessor.Builder()
        .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
        .build()
    val processors = listOf<VisionProcessor>(
        aprilTagProcessor
    )

    val camera1: VisionPortal =
        VisionPortal
            .Builder()
            .setCamera(opMode.hardwareMap["cam1"] as WebcamName)
            .addProcessors(*processors.toTypedArray())
            .setCameraResolution(Size(cameraSizeX.toInt(), cameraSizeY.toInt()))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .build()

    var targetPose: Pose2d? = null
    var angleError: Double = 0.0
    var aprilTags = listOf<AprilTagDetection>()

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            while (opModeIsActive() && !isStopRequested) {
                aprilTags = aprilTagProcessor.detections
                telemetry.addData("april tags", aprilTags.size)
                val firstTag = aprilTags.firstOrNull()
                if (firstTag != null) {
                    telemetry.addData("dist", firstTag.ftcPose.range)
                    telemetry.addData("heading", -firstTag.ftcPose.bearing)

                    val pose = Pose2d() // replace with odom
                    val absoluteHeading = pose.radians + Math.toRadians(-firstTag.ftcPose.bearing)
                    angleError = -firstTag.ftcPose.bearing
                    targetPose = pose + Vec2d(firstTag.ftcPose.range, 0).rotate(absoluteHeading)
                }
                telemetry.update()
                sync()
            }
        }
    }

    override val command = Command(this.name, cleanup, run)

    companion object {
        @JvmField
        var cameraSizeX = 1280

        @JvmField
        var cameraSizeY = 720
    }
}