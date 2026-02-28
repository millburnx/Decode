package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.util.Pose2d
import com.millburnx.util.vector.Vec2d
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import java.util.*

@Configurable
class AutoAdjust(
    val opMode: OpMode,
    val flyWheel: FlyWheel,
    val hood: Hood,
    val getPose: () -> Pose2d,
    val isRed: Boolean = true
) :
    Subsystem("AutoAdjust") {

    var forceFar: Boolean = false

    var minRapidRPM: Double = 0.0

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            with(opMode) {
                val goal = if (isRed) redGoal else blueGoal
                val distance = getPose().distanceTo(goal)

                if (useRecoil) {
                    val target = getRecoilTarget(distance, flyWheel.rpm)

                    minRapidRPM = target.minRPM

                    flyWheel.shootingRPM = target.maxRPM
                    hood.target = target.targetHood

                    tel.addData("aa | min rpm", target.minRPM)
                    tel.addData("aa | target rpm", target.maxRPM)
                    tel.addData("aa | target angle", target.targetHood)
                } else {
                    val target = getTarget(distance, forceFar)
                    minRapidRPM = target.first

                    flyWheel.shootingRPM = target.first
                    hood.target = target.second
                    tel.addData("aa | target rpm", target.first)
                    tel.addData("aa | target angle", target.second)
                }
                tel.addData("aa | dist", distance)
            }
        }
    }

    companion object {
        val redGoal = Vec2d(144.0 - 6.0, 144.0 - 6.0)
        val blueGoal = Vec2d(6.0, 144.0 - 6.0)

        fun getRecoilTarget(distance: Distance, currentRPM: RPM): TargetData {
            val data: TreeMap<RPM, Angle> = if (distance < FAR_DISTANCE) {
                // always round up
                (CLOSE_DATA.ceilingEntry(distance) ?: CLOSE_DATA.lastEntry()).value
            } else {
                FAR_DATA
            }

            val minRPM = data.firstKey()
            val maxRPM = data.lastKey()

            val angle = (data.floorEntry(currentRPM - rpmCompensation) ?: data.firstEntry()).value
            return TargetData(minRPM, maxRPM, angle)
        }

        fun getTarget(distance: Distance, forceFar: Boolean = false): Pair<RPM, Angle> {
            return if (distance < FAR_DISTANCE && !forceFar) {
                // always round up
                (NO_RECOIL_DATA.ceilingEntry(distance) ?: NO_RECOIL_DATA.lastEntry()).value
            } else {
                FAR_ZONE
            }
        }

        @JvmField
        var rpmCompensation = 130

        @JvmField
        var useRecoil = false
    }
}

/**
 * @property[minRPM] The minimum rpm needed to fire at this distance
 * @property[maxRPM] The maximum rpm for this distance, this should be your target rpm
 * @property[targetHood] The target hood angle for this RPM and distance
 */
data class TargetData(
    val minRPM: RPM, val maxRPM: RPM, val targetHood: Angle
)