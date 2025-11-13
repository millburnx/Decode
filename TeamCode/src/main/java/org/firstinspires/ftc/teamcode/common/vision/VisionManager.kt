package org.firstinspires.ftc.teamcode.common.vision

import android.util.Size
import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor

@Suppress("SpreadOperator")
@Configurable
object VisionManager {
    val processors = mutableListOf<VisionProcessor>()

    fun init() {
        processors.clear()
    }

    fun build(opMode: OpMode): VisionPortal {
//        val panelsIntegration = PanelsIntegration()
        val portal = VisionPortal
            .Builder()
            .setCamera(opMode.hardwareMap["cam1"] as WebcamName)
            .addProcessors(*processors.toTypedArray())
            .setCameraResolution(Size(cameraSizeX, cameraSizeY))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .build()
//        PanelsCameraStream.startStream(panelsIntegration)
        return portal
    }

    @JvmField
    var cameraSizeX = 640

    @JvmField
    var cameraSizeY = 360
}