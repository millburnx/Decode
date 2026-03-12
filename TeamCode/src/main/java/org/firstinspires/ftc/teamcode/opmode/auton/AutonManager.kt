package org.firstinspires.ftc.teamcode.opmode.auton


import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.paths.PedroBuilder
import com.millburnx.cmdxpedro.paths.PedroLoader
import com.millburnx.cmdxpedro.paths.heading.HeadingInterpolation
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.pedroparser.types.Sequence
import com.millburnx.util.Pose2d
import com.pedropathing.paths.PathBuilder
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.opmode.OpMode

open class BaseAutonManager(val opMode: OpMode, val drive: Drive, val isMirrored: Boolean = false) {
    val builder: PedroBuilder = PedroBuilder(isMirrored)
}

class AutonManager(opMode: OpMode, drive: Drive, val sequenceName: String, isMirrored: Boolean = false): BaseAutonManager(opMode, drive, isMirrored) {
    val loadedSequence: Sequence = PedroLoader.load("Paths/$sequenceName.pp")
    val loadedPath: List<Pair<com.millburnx.cmdxpedro.paths.path.Path, HeadingInterpolation>>
        get() = PedroLoader.sequenceToPath(loadedSequence)
    val startingPose: Pose2d
        get() = loadedSequence.startPose.mirror(isMirrored)

    init {
        println("starting pose: $startingPose, ${loadedSequence.startPose}")
        drive.pose = startingPose
    }

    fun runPath(index: Int, maxPower: Double = 1.0, callbacks: (PathBuilder) -> Unit = {}): Command =
        builder.PathCommand(
            drive.follower,
            loadedPath[index].first,
            loadedPath[index].second,
            { !opMode.isStopRequested },
            maxPower = maxPower,
            pathCallback = callbacks
        )
}