package org.firstinspires.ftc.teamcode.opmode.auton


import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.paths.PedroBuilder
import com.millburnx.cmdxpedro.paths.PedroLoader
import com.millburnx.cmdxpedro.paths.heading.HeadingInterpolation
import com.millburnx.pedroparser.types.Sequence
import com.millburnx.util.Pose2d
import com.pedropathing.follower.Follower
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants

// TODO: create the pedro subsystem and pass it so we can automatically the starting pose
//  (find a way to do it cleanly since this is created after it)
//  i think we let auton manager create pedro, drive can have pedro passed in. if not it'll create itself
class AutonManager(val opMode: OpMode, val sequenceName: String, val isMirrored: Boolean = false) {
    val builder: PedroBuilder = PedroBuilder(isMirrored)
    val loadedSequence: Sequence = PedroLoader.load("paths/$sequenceName.pp")
    val loadedPath: List<Pair<com.millburnx.cmdxpedro.paths.path.Path, HeadingInterpolation>>
        get() = PedroLoader.sequenceToPath(loadedSequence)
    val startingPose: Pose2d
        get() = loadedSequence.startPose

    val follower: Follower = Constants.createManualFusionFollower(opMode.hardwareMap, { opMode.deltaTime }).apply {
        setStartingPose(startingPose.toPedro())
    }

    fun runPath(index: Int, maxPower: Double = 1.0): Command =
        builder.PathCommand(follower, loadedPath[index].first, loadedPath[index].second, { !opMode.isStopRequested })
}