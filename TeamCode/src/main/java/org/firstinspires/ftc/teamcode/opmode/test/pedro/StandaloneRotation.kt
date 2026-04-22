package org.firstinspires.ftc.teamcode.opmode.test.pedro

import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.control.PIDFController
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.abs
import kotlin.math.sign

class StandaloneRotation(
    val headingPIDFCoefficients: () -> PIDFCoefficients = { Constants.followerConstants.coefficientsHeadingPIDF },
    val secondaryHeadingPIDFCoefficients: () -> PIDFCoefficients = { Constants.followerConstants.coefficientsSecondaryHeadingPIDF },
    val useSecondaryHeadingPIDF: () -> Boolean = { Constants.followerConstants.useSecondaryHeadingPIDF },
    val headingPIDFSwitch: () -> Double = { Constants.followerConstants.headingPIDFSwitch }
) {
    val pidf = PIDFController(headingPIDFCoefficients())
    val secondaryPidf = PIDFController(secondaryHeadingPIDFCoefficients())

    fun calc(current: Double, target: Double): Double {
        pidf.coefficients = headingPIDFCoefficients()
        secondaryPidf.coefficients = secondaryHeadingPIDFCoefficients()

//        val direction = MathFunctions.getTurnDirection(current, target)
//        val magnitude = MathFunctions.getSmallestAngleDifference(current, target);
//        val headingError = direction * magnitude

        val headingError = target - current

        val useSecondaryPIDF = useSecondaryHeadingPIDF()
        val secondaryPIDFThreshold = headingPIDFSwitch()
        if (useSecondaryPIDF && abs(headingError) < secondaryPIDFThreshold) {
            secondaryPidf.updateFeedForwardInput(sign(headingError))
            secondaryPidf.updateError(headingError)
            return secondaryPidf.run()
        }
        pidf.updateFeedForwardInput(sign(headingError))
        pidf.updateError(headingError)
        return pidf.run()
    }
}