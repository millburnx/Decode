package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants

class Pedro(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Pedro") {
    val follower = Constants.createFollower(opMode.hardwareMap).apply {
        setStartingPose(Pose(0.0, 0.0, 0.0))
    }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            follower.update()
            if (isTeleop) follower.startTeleopDrive()
            WaitFor { isStarted || !isStopRequested }
            while (!isStopRequested) {
                follower.update()
                if (isTeleop) {
                    follower.setTeleOpDrive(
                        -gp1.left_stick_y.toDouble(),
                        -gp1.left_stick_x.toDouble(),
                        -gp1.right_stick_x.toDouble()
                    )
                }
                sync()
            }
        }
    }
}