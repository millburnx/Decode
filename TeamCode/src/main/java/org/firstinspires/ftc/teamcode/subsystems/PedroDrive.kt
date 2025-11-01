package org.firstinspires.ftc.teamcode.subsystems

import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Pedro.Constants
import org.firstinspires.ftc.teamcode.util.Pose2d
import org.firstinspires.ftc.teamcode.util.toPedro

class PedroDrive(opMode: LinearOpMode, startingPose: Pose2d = Pose2d()) : Subsystem("Pedro Drive") {

    val follower = Constants.createFollower(opMode.hardwareMap).apply {
        setStartingPose(startingPose.toPedro())
    }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            while (opModeIsActive() && !isStopRequested) {
                follower.update()
                sync()
            }
        }
    }

    override val command = Command(this.name, cleanup, run)
}