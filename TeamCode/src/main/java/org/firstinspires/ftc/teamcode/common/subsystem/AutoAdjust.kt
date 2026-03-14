package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class AutoAdjust(
    val opMode: OpMode,
    val flyWheel: FlyWheel,
    val hood: Hood,
    val getPose: () -> Pose2d,
    val getVelocity: () -> Vec2d,
    val isRed: Boolean = true
) :
    Subsystem("AutoAdjust") {
    var minRapidRPM: Double = 0.0

    var turretAngle = 0.0

    var enabled = true

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            with(opMode) {
                if (!enabled) return@OpModeLoop
                val goal = goal.mirror(isRed)

                val SOTMResults = SOTM.calculate(opMode.tel, getVelocity(), getPose(), goal)

                val rawDistance = getPose().distanceTo(goal)
                val distance = if (useSOTM) SOTMResults.first else rawDistance
                if (useTelemetry) tel.addData("distance!", distance)
                turretAngle = if (useSOTM) SOTMResults.second else getPose().position.angleTo(goal).toDegrees()

                val target = getTarget(distance)
                minRapidRPM = target.first

                flyWheel.shootingRPM = target.first
                hood.target = target.second

                if (useTelemetry) {
                    tel.addData("aa | target rpm", target.first)
                    tel.addData("aa | target angle", target.second)
                    tel.addData("aa | dist", rawDistance)
                }
            }
        }
    }

    companion object {
        val goal = Vec2d(-0.0, 144.0)

        fun getTarget(distance: Distance): Pair<RPM, Angle> {
            return (DATA.ceilingEntry(distance) ?: DATA.lastEntry()).value
        }

        @JvmField
        var useSOTM = true

        @JvmField
        var useTelemetry = false
    }
}