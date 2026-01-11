package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.Style
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.util.Pose2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Drawing


@Configurable
class Limelight(opMode: OpMode, val getHeading: (() -> Double)? = null) : Subsystem("Limelight") {
    @Suppress("MemberNameEqualsClassName")
    val limelight = (opMode.hardwareMap["limelight"] as Limelight3A).apply {
        pipelineSwitch(0)
        start()
    }

    var llPose: Pose2d? = null

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                val heading = getHeading?.invoke() ?: 0.0
                limelight.updateRobotOrientation(normalizeDegrees(heading - 90));
                val result = limelight.latestResult
                println("ll log $result")
                if (result != null) {
                    if (result.isValid) {
                        val pose = if (useMT2) result.botpose_MT2 else result.botpose
                        val inches = pose.position.toUnit(DistanceUnit.INCH)
                        val fixedPose = Pose2d(
                            72.0 + inches.y,
                            72.0 - inches.x,
                            90.0 + pose.orientation.getYaw(AngleUnit.DEGREES)
                        )
                        Drawing.drawRobot(
                            fixedPose.toPedro(), Style(
                                "", "#42d44b", 0.75
                            )
                        )
                        llPose = fixedPose
                        tel.addData("ll | tx", result.tx)
                        tel.addData("ll | ty", result.ty)
                        tel.addData("ll | pose", fixedPose)
                    }
                }
                sync()
            }
        }
    }

    companion object {
        @JvmField
        var useMT2 = false
    }
}