package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.Style
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.pedro.Drawing

@Configurable
class Drive(
    val opMode: OpMode,
    var isTeleop: Boolean = false,
    val startingPose: Pose2d = Pose2d(),
    val follower: Follower = Constants.createManualFollower(opMode.hardwareMap).apply {
        setStartingPose(startingPose.toPedro())
    }
) : Subsystem("Drive") {

    val pose
        get() = Pose2d(follower.pose.x, follower.pose.y, follower.pose.heading.toDegrees())

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            val m0 = hardwareMap["m0"] as DcMotorEx
            val m1 = hardwareMap["m1"] as DcMotorEx
            val m2 = hardwareMap["m2"] as DcMotorEx
            val m3 = hardwareMap["m3"] as DcMotorEx

            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                follower.update()
                Drawing.drawRobot(
                    pose.toPedro(), Style(
                        "", "#3F51B5", 0.75
                    )
                )
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
//                tel.addData("m0 current", m0.getCurrent(CurrentUnit.AMPS))
//                tel.addData("m1 current", m1.getCurrent(CurrentUnit.AMPS))
//                tel.addData("m2 current", m2.getCurrent(CurrentUnit.AMPS))
//                tel.addData("m3 current", m3.getCurrent(CurrentUnit.AMPS))
                tel.addData("pose", pose)
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var useFloat = true

        @JvmField
        var useFieldCentric = false
    }
}