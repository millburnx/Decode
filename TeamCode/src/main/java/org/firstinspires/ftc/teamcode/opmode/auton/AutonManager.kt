package org.firstinspires.ftc.teamcode.opmode.auton


import com.millburnx.cmdxpedro.paths.PedroBuilder
import com.millburnx.cmdxpedro.paths.PedroLoader
import com.millburnx.pedroparser.types.Path
import com.millburnx.pedroparser.types.Sequence
import com.millburnx.util.Pose2d

// TODO: create the pedro subsystem and pass it so we can automatically the starting pose
//  (find a way to do it cleanly since this is created after it)
//  i think we let auton manager create pedro, drive can have pedro passed in. if not it'll create itself
public class AutonManager(public val isMirrored: Boolean = false, public val sequenceName: String) {
    public val builder: PedroBuilder = PedroBuilder(isMirrored)
    public val loadedSequence: Sequence = PedroLoader.load(sequenceName)
    public val loadedPaths: List<Path>
        get() = loadedSequence.lines
    public val loadedPathsMap: Map<String, Path>
        get() = loadedPaths.associateBy { it.name }
    public val startingPose: Pose2d
        get() = loadedSequence.startPose
}