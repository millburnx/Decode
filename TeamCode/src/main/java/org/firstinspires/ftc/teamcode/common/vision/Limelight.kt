package org.firstinspires.ftc.teamcode.common.vision

import com.millburnx.cmdx.Command
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.common.subsystem.Pedro
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.common.toPose
import org.firstinspires.ftc.teamcode.opmode.OpMode


class Limelight(val opMode: OpMode, val pedro: Pedro) : Subsystem("Limelight") {
    val limelight = opMode.hardwareMap.get(Limelight3A::class.java, "limelight").apply {
        setPollRateHz(100)
        pipelineSwitch(0)
        start()
    }

    var result: LLResult? = null
    var pipeline: Int = 0

    override val run: suspend Command.() -> Unit = {
        with (opMode) {
            while (!isStopRequested) {
                limelight.updateRobotOrientation(pedro.pose.heading)
                val result = limelight.latestResult
                if (result == null || !result.isValid) {
                    this@Limelight.result = null
                    continue
                }
                this@Limelight.result = result
                this@Limelight.pipeline = result.pipelineIndex

                when (pipeline) {
                    0 -> parseApriltag(result)
                    1 -> {
                        println("Limelight: unknown pipeline")
                    }
                }
                sync()
            }
        }
    }

    fun parseApriltag(result: LLResult) {
        with (opMode) {
            val mt2 = result.botpose_MT2.toPose()
            tel.addData("ll pose2", mt2)
            val mt = result.botpose.toPose()
            tel.addData("ll pose", mt)
        }
    }
}