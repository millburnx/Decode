package org.firstinspires.ftc.teamcode.common

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose

@Configurable
object GlobalStore {
    var autonPose: Pose? = null // if this is null when starting teleop, robot restarted

    @JvmField
    var useTelemetry: Boolean = false
}