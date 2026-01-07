package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants

@Configurable
class Drive(val opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Drive") {
    val follower = Constants.createFollower(opMode.hardwareMap).apply {
        setStartingPose(Pose(0.0, 0.0))
    }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            val m0 = hardwareMap["m0"] as DcMotorEx
            val m1 = hardwareMap["m1"] as DcMotorEx
            val m2 = hardwareMap["m2"] as DcMotorEx
            val m3 = hardwareMap["m3"] as DcMotorEx

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
                tel.addData("m0 current", m0.getCurrent(CurrentUnit.AMPS))
                tel.addData("m1 current", m1.getCurrent(CurrentUnit.AMPS))
                tel.addData("m2 current", m2.getCurrent(CurrentUnit.AMPS))
                tel.addData("m3 current", m3.getCurrent(CurrentUnit.AMPS))
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