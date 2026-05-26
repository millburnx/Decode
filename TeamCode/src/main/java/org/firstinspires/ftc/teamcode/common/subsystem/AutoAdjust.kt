package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.config.core.util.ShotPlanner
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
class AutoAdjust(
    val opMode: OpMode,
    val flyWheel: FlyWheel,
    val hood: Hood,
    val getPose: () -> Pose2d,
    val getVelocity: () -> Vec2d,
    val getOmegaDeg: () -> Double = { 0.0 },  // angular velocity in deg/s; wire up if available
    val isFarZone: () -> Boolean = { false },
    val isRed: Boolean = true,
    val rpmOffset: () -> Double = { 0.0 },
) : Subsystem("AutoAdjust") {
    val planner = ShotPlanner(opMode.tel)

    var turretAngle = 0.0
    var enabled = true

    /** Exposed so readiness gate elsewhere can check RPM tolerance */
    var minRapidRPM: Double = 0.0

    override val run: suspend Command.() -> Unit = {
        OpModeLoop(opMode) {
            with(opMode) {
                if (!enabled) return@OpModeLoop

                val rawGoal  = (if (isFarZone()) goalFar else goal).mirror(isRed)
                val pose     = getPose()
                val velocity = getVelocity()

                val cmd = planner.plan(
                    robotPose   = pose,
                    vx          = velocity.x,
                    vy          = velocity.y,
                    omegaDeg    = getOmegaDeg(),
                    realGoal    = rawGoal,
                    measuredRpm = flyWheel.shootingRPM,
                )

                tel.addData("lead angle",
                    normalizeDegrees(cmd.turretAngleDeg(pose.position) - pose.position.angleTo(rawGoal).toDegrees())
                )

                val turretPose = pose.position + Vec2d(turretX, turretY).rotate(pose.radians) / 25.4

                if (cmd.possible) {
                    turretAngle          = cmd.turretAngleDeg(turretPose)
                    minRapidRPM          = cmd.targetRpm + rpmOffset()
                    flyWheel.shootingRPM = cmd.targetRpm + rpmOffset()
                    hood.target          = cmd.hoodNorm
                } else {
                    // Fallback — raw distance, ceiling entry from table, aim at real goal
                    val rawDist  = pose.distanceTo(rawGoal)
                    val fallback = DATA.ceilingEntry(rawDist)?.value ?: DATA.lastEntry().value
                    turretAngle          = turretPose.angleTo(rawGoal).toDegrees()
                    minRapidRPM          = fallback.first + rpmOffset()
                    flyWheel.shootingRPM = fallback.first + rpmOffset()
                    hood.target          = fallback.second
                }

                if (useTelemetry) {
                    tel.addData("aa | dist",       "%.1f pu".format(cmd.distancePoseUnits))
                    tel.addData("aa | target rpm", cmd.targetRpm)
                    tel.addData("aa | hood norm",  "%.2f  (%.1f°)".format(cmd.hoodNorm, cmd.hoodDeg))
                    tel.addData("aa | turret",     "%.1f°".format(turretAngle))
                    tel.addData("aa | horizon",    "%.0f ms".format(cmd.totalHorizonSec * 1000.0))
                    tel.addData("aa | highAccel",  cmd.highAccelWarning)
                    tel.addData("aa | possible",   cmd.possible)
                }
            }
        }
    }

    companion object {
        @JvmField var goalXOffsetClose = 0.0
        @JvmField var goalYOffsetClose = -5.0
        @JvmField var goalXOffsetFar   = 5.0
        @JvmField var goalYOffsetFar   = 0.0

        val goal    get() = Vec2d(0.0 + goalXOffsetClose, 144.0 + goalYOffsetClose)
        val goalFar get() = Vec2d(0.0 + goalXOffsetFar,   144.0 + goalYOffsetFar)

        @JvmField var useTelemetry = true

        @JvmField var turretX = -42.0
        @JvmField var turretY = -16.0
    }
}