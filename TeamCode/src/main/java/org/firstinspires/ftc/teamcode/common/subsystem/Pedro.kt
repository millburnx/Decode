package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.toDegrees
import com.millburnx.cmdxpedro.util.toRadians
import com.pedropathing.control.PIDFController
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.common.normalizeDegrees
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.abs


class Pedro(opMode: OpMode, var isTeleop: Boolean = false) : Subsystem("Pedro") {
    val follower = Constants.createFollower(opMode.hardwareMap).apply {
        setStartingPose(Pose(0.0, 0.0, 0.0))
    }

    val headingLock = HeadingLock(this)
    var isLocked = false;

    val pose: Pose2d
        get() = Pose2d(follower.pose.x, follower.pose.y, follower.pose.heading.toDegrees())

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            follower.update()
            if (isTeleop) follower.startTeleopDrive(true)
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                follower.update()
                if (isTeleop) {
                    if (isLocked) {
                        follower.setTeleOpDrive(
                            -gp1.left_stick_y.toDouble(),
                            -gp1.left_stick_x.toDouble(),
                            headingLock.calc().magnitude,
                            false
                        )
                    } else {
                        follower.setTeleOpDrive(
                            -gp1.left_stick_y.toDouble(),
                            -gp1.left_stick_x.toDouble(),
                            -gp1.right_stick_x.toDouble()
                        )
                    }
                }
                sync()
            }
        }
    }
}

class HeadingLock(val pedro: Pedro) {
    private val constants = pedro.follower.constants

    private val headingPIDF = PIDFController(constants.coefficientsHeadingPIDF)
    private val secondaryHeadingPIDF = PIDFController(constants.coefficientsSecondaryHeadingPIDF)
    private val headingPIDFSwitch = constants.headingPIDFSwitch
    var targetHeading = 0.0

    fun calc(): Vector {
        val headingError = normalizeDegrees(targetHeading - pedro.pose.heading).toRadians()
        if (abs(headingError) < headingPIDFSwitch &&
            constants.useSecondaryHeadingPIDF
        ) {
            secondaryHeadingPIDF.updateFeedForwardInput(
                MathFunctions.getTurnDirection(
                    pedro.pose.radians,
                    targetHeading.toRadians()
                )
            )
            secondaryHeadingPIDF.updateError(headingError)
            val headingVector = Vector(
                secondaryHeadingPIDF.run().coerceIn(-1.0, 1.0),
                pedro.pose.radians
            )
            return headingVector.copy()
        }

        headingPIDF.updateFeedForwardInput(
            MathFunctions.getTurnDirection(
                pedro.pose.radians,
                targetHeading.toRadians()
            )
        )
        headingPIDF.updateError(headingError)
        val headingVector =
            Vector(headingPIDF.run().coerceIn(-1.0, 1.0), pedro.pose.radians)
        return headingVector.copy()
    }
}