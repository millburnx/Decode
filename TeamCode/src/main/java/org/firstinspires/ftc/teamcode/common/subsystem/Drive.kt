package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants


class Drive(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Drive") {
    val follower = Constants.createFollower(opMode.hardwareMap).apply {
        setStartingPose(Pose(0.0, 0.0))
    }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                follower.update()
                if (isTeleop) {
                    if (!follower.teleopDrive) {
                        follower.startTeleopDrive(!useFloat)
                    }
                    follower.setTeleOpDrive(
                        -gp1.current.leftJoyStick.y,
                        -gp1.current.leftJoyStick.x,
                        -gp1.current.rightJoyStick.x,
                        !useFieldCentric // Robot Centric
                    );
                }
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var useFloat = false
        @JvmField
        var useFieldCentric = false
    }
}