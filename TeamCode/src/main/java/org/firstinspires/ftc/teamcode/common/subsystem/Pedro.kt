package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.toDegrees
import com.millburnx.cmdxpedro.util.toRadians
import com.pedropathing.control.PIDFController
import com.pedropathing.math.MathFunctions
import org.firstinspires.ftc.teamcode.common.hardware.gamepad.SlewRateLimiter
import org.firstinspires.ftc.teamcode.common.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.simplifiedString
import org.firstinspires.ftc.teamcode.common.toPedro
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sign

@Configurable
class Pedro(val opMode: OpMode, val startingPose: Pose2d = Pose2d(), var isTeleop: Boolean = false) :
    Subsystem("Pedro") {
    val follower = Constants.createFollower(opMode.hardwareMap).apply {
        setStartingPose(startingPose.toPedro())
    }
    var isLocked = false;

    val headingController = HeadingLock(this)

    val pose: Pose2d
        get() = Pose2d(follower.pose.x, follower.pose.y, follower.pose.heading.toDegrees())

    override val run: suspend Command.() -> Unit = {
        var prevX = 0.0
        var prevY = 0.0

        with(opMode) {
            follower.update()
            if (isTeleop) follower.startTeleopDrive(false)
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                follower.update()
                if (isTeleop && !isLocked) {
                    // axis snapping
                    val processedX = processInput(prevX, gp1.current.leftJoyStick.x)
                    val processedY = processInput(prevY, gp1.current.leftJoyStick.y)
                    if (!useAbsoluteHeading || gp1.current.rightJoyStick.vector.magnitude() < 0.2) {
                        val rx = gp1.current.rightJoyStick.x
                        follower.setTeleOpDrive(
                            -processedY,
                            -processedX,
                            abs(rx).pow(turningCurve).coerceIn(kS, 1.0) * sign(-rx) * turnScaling
                        )
                    } else {
                        val targetHeading = atan2(-gp1.current.rightJoyStick.y, gp1.current.rightJoyStick.x).toDegrees()
                        headingController.targetHeading = targetHeading
                        follower.setTeleOpDrive(
                            -processedY, -processedX, headingController.calc()
                        )
                        tel.addData("target heading", targetHeading)
                    }
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
            prevSnapped, newSnapped, maxRate * dt
        )

        return abs(slewedResult.pow(driveCurve)).coerceIn(kS, 1.0) * sign(slewedResult)
    }

    companion object {
        @JvmField
        var snapThreshold = 0.1;

        @JvmField
        var maxRate = 4.0;

        @JvmField
        var kS = 0.03

        @JvmField
        var turnScaling = 0.5

        @JvmField
        var driveCurve = 2.0

        @JvmField
        var turningCurve = 1.5

        @JvmField
        var useAbsoluteHeading = false
    }
}


class HeadingLock(val pedro: Pedro) {
    private val constants = pedro.follower.constants

    private val headingPIDF = PIDFController(constants.coefficientsHeadingPIDF)
    private val secondaryHeadingPIDF = PIDFController(constants.coefficientsSecondaryHeadingPIDF)
    private val headingPIDFSwitch = constants.headingPIDFSwitch
    var targetHeading = 0.0

    fun calc(): Double {
        val headingError = normalizeDegrees(targetHeading - pedro.pose.heading).toRadians()
        val activePID = if (constants.useSecondaryHeadingPIDF && abs(headingError) < headingPIDFSwitch) {
            secondaryHeadingPIDF
        } else {
            headingPIDF
        }
        activePID.updateFeedForwardInput(
            MathFunctions.getTurnDirection(
                pedro.pose.radians, targetHeading.toRadians()
            )
        )
        activePID.updateError(headingError)
        return activePID.run().coerceIn(-1.0, 1.0)
    }
}