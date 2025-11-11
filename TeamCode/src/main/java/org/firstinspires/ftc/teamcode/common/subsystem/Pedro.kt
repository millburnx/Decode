package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.toDegrees
import org.firstinspires.ftc.teamcode.common.hardware.gamepad.SlewRateLimiter
import org.firstinspires.ftc.teamcode.common.simplifiedString
import org.firstinspires.ftc.teamcode.common.toPedro
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

@Configurable
class Pedro(val opMode: OpMode, val startingPose: Pose2d = Pose2d(), var isTeleop: Boolean = false) : Subsystem("Pedro") {
    val follower = Constants.createFollower(opMode.hardwareMap).apply {
        setStartingPose(startingPose.toPedro())
    }
    var isLocked = false;

    val pose: Pose2d
        get() = Pose2d(follower.pose.x, follower.pose.y, follower.pose.heading.toDegrees())

    override val run: suspend Command.() -> Unit = {
        var prevX = 0.0
        var prevY = 0.0

        with(opMode) {
            follower.update()
            if (isTeleop) follower.startTeleopDrive(true)
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                follower.update()
                if (isTeleop && !isLocked) {
                    // axis snapping
                    val processedX = processInput(prevX, gp1.current.leftJoyStick.x)
                    val processedY = processInput(prevY, gp1.current.leftJoyStick.y)
                    val rx = gp1.current.rightJoyStick.x
                    follower.setTeleOpDrive(
                        -processedY,
                        -processedX,
                        -(abs(rx.pow(2)) * sign(rx))
                    )
                    tel.addData("px", processedX)
                    tel.addData("py", processedY)

                    prevX = processedX
                    prevY = processedY
                }
                tel.addData("pose", pose.simplifiedString())
                sync()
            }
        }
    }

    private fun snapTo(raw: Double, target: Double, threshold: Double): Double {
        if (abs(raw - target) < threshold) return target
        return raw
    }

    private fun processInput(prevValue: Double, newValue: Double): Double {
        val prevSnapped = snapTo(prevValue, 0.0, snapThreshold)
        val newSnapped = snapTo(newValue, 0.0, snapThreshold)

        val dt = opMode.deltaTime
        val slewedResult = SlewRateLimiter.limit(
            prevSnapped,
            newSnapped,
            maxRate * dt
        )

        return abs(slewedResult.pow(2)) * sign(slewedResult)
    }

    companion object {
        @JvmField
        var snapThreshold = 0.1;

        @JvmField
        var maxRate = 0.0;
    }
}