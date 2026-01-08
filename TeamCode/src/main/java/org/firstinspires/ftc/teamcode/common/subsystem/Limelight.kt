package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.Pose2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.opmode.OpMode


@Configurable
class Limelight(opMode: OpMode, val turret: Turret?) : Subsystem("Limelight") {
    @Suppress("MemberNameEqualsClassName")
    val limelight = (opMode.hardwareMap["limelight"] as Limelight3A).apply {
        pipelineSwitch(0)
        start()
    }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                val heading = turret?.totalHeading ?: 0.0
                limelight.updateRobotOrientation(heading);
                val result = limelight.latestResult
                if (result != null) {
                    if (result.isValid) {
                        val pose = result.botpose
                        val poseMT2 = result.botpose_MT2
                        val inches = pose.position.toUnit(DistanceUnit.INCH)
                        val inchesMT2 = poseMT2.position.toUnit(DistanceUnit.INCH)
                        val fixedPose = Pose2d(inches.x, inches.y, pose.orientation.getYaw(AngleUnit.DEGREES))
                        val fixedPoseMT2 =
                            Pose2d(inchesMT2.x, inchesMT2.y, poseMT2.orientation.getYaw(AngleUnit.DEGREES))
                        tel.addData("ll | tx", result.tx)
                        tel.addData("ll | ty", result.ty)
                        tel.addData("ll | pose", fixedPose)
                        tel.addData("ll | pose mt2", fixedPoseMT2)
                    }
                }
                sync()
            }
        }
    }

    companion object {

    }
}