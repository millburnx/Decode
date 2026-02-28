package org.firstinspires.ftc.teamcode.common

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.util.Pose2d

@Configurable
object GlobalStore {
    var autonPose: Pose2d? = null // if this is null when starting teleop, robot restarted

    @JvmField
    var useTelemetry: Boolean = false
}