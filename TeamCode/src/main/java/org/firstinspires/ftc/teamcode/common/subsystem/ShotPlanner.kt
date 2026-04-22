package org.firstinspires.ftc.teamcode.config.core.util

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.TelemetryManager
import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.subsystem.DATA
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

@Configurable
class ShotPlanner(
    val tel: TelemetryManager
) {

    // ─────────────────────────────────────────────────────────────────────────
    // Configurable constants
    // ─────────────────────────────────────────────────────────────────────────

    companion object {
        // Physical constants
        @JvmField var POSE_UNITS_TO_METERS   = 0.0254
        @JvmField var EXIT_VEL_M_PER_RPM     = 0.00365

        // Hood angle mapping  (0.0 = MIN_HOOD_DEG, 1.0 = MAX_HOOD_DEG)
        @JvmField var MIN_HOOD_DEG           = 35.0
        @JvmField var MAX_HOOD_DEG           = 60.0

        // Prediction horizon = SERVO_DELAY_SEC + t_flight (iterated)
        @JvmField var SERVO_DELAY_SEC        = 0.20
        @JvmField var SOTM_ITERS             = 3
        @JvmField var MAX_LEAD_POSE_UNITS    = 100.0

        // RPM output cap
        @JvmField var MAX_RPM                = 4300.0

        // RPM rolling-average window (setpoint smoothing)
        @JvmField var RPM_SMOOTH_WINDOW      = 8

        // EMA alpha for acceleration  (0 < α ≤ 1, higher = more responsive)
        @JvmField var ACCEL_ALPHA            = 0.20

        // If linear accel magnitude exceeds this, highAccelWarning is set.
        // Caller decides whether to inhibit the indexer.
        @JvmField var ACCEL_WARN_POSE_PER_S2 = 120.0

        @JvmField var SOTM_ENABLED           = true

        @JvmField var leadMultiplier = 3.0

        fun enableSOTM()  { SOTM_ENABLED = true  }
        fun disableSOTM() { SOTM_ENABLED = false }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Output value object
    // ─────────────────────────────────────────────────────────────────────────

    data class ShotCommand(
        /** Where the robot will be when the ball leaves the launcher */
        val predictedPose: Pose2d,
        /** Shifted goal the turret should aim at to compensate for robot motion */
        val virtualGoal: Vec2d,
        val distancePoseUnits: Double,
        val targetRpm: Double,
        /** Hood setpoint normalised 0–1  (0 = MIN_HOOD_DEG, 1 = MAX_HOOD_DEG) */
        val hoodNorm: Double,
        /** Total prediction horizon: servo delay + flight time */
        val totalHorizonSec: Double,
        /** False if distance is below the minimum (40 pu) */
        val possible: Boolean,
        /** True if accel magnitude exceeded ACCEL_WARN threshold */
        val highAccelWarning: Boolean,
        val axPosePerS2: Double,
        val ayPosePerS2: Double,
        /** Angular acceleration in rad/s² */
        val alphaRadPerS2: Double,
    ) {
        val hoodDeg: Double get() = MIN_HOOD_DEG + hoodNorm * (MAX_HOOD_DEG - MIN_HOOD_DEG)

        /** Turret angle to virtual goal in degrees, from the predicted robot pose */
        fun turretAngleDeg(robotPos: Vec2d) = robotPos.angleTo(virtualGoal).toDegrees()
    }

    // ─────────────────────────────────────────────────────────────────────────
    // DATA TreeMap lookup — interpolation within range, extrapolation beyond
    //
    // Below 40 pu : hard clamp to table floor (physically impossible to shoot)
    // 40 – 148 pu : linear interpolation between bracketing entries
    // Above 148 pu: linear extrapolation from last two entries,
    //               RPM capped at MAX_RPM, hoodNorm clamped 0–1
    // ─────────────────────────────────────────────────────────────────────────

    private fun lookup(dist: Double): Pair<Double, Double> {
        val minKey = DATA.firstKey()
        val maxKey = DATA.lastKey()

        if (dist <= minKey) {
            val v = DATA.firstEntry().value
            return v.first.coerceIn(0.0, MAX_RPM) to v.second.coerceIn(0.0, 1.0)
        }

        if (dist <= maxKey) {
            val lo = DATA.floorEntry(dist)!!
            val hi = DATA.ceilingEntry(dist)!!
            if (lo.key == hi.key) return lo.value.first to lo.value.second
            val t     = (dist - lo.key) / (hi.key - lo.key)
            val rpm   = lo.value.first  + t * (hi.value.first  - lo.value.first)
            val angle = lo.value.second + t * (hi.value.second - lo.value.second)
            return rpm.coerceIn(0.0, MAX_RPM) to angle.coerceIn(0.0, 1.0)
        }

        // Extrapolate from last two entries
        val hi   = DATA.lastEntry()
        val lo   = DATA.lowerEntry(maxKey)!!
        val span = hi.key - lo.key
        if (span < 1e-6) return hi.value.first.coerceIn(0.0, MAX_RPM) to hi.value.second.coerceIn(0.0, 1.0)
        val t     = (dist - lo.key) / span
        val rpm   = lo.value.first  + t * (hi.value.first  - lo.value.first)
        val angle = lo.value.second + t * (hi.value.second - lo.value.second)
        return rpm.coerceIn(0.0, MAX_RPM) to angle.coerceIn(0.0, 1.0)
    }

    private fun isPossible(dist: Double) = dist >= DATA.firstKey()

    // ─────────────────────────────────────────────────────────────────────────
    // Flight time estimate
    //
    // hoodNorm 0→1 maps to MIN→MAX hood deg from vertical,
    // i.e. (90 - MIN)→(90 - MAX) elevation angle.
    // ─────────────────────────────────────────────────────────────────────────

    private fun flightTimeSec(dist: Double, rpm: Double, hoodNorm: Double): Double {
        val x       = dist * POSE_UNITS_TO_METERS
        val elevDeg = 90.0 - (MIN_HOOD_DEG + hoodNorm * (MAX_HOOD_DEG - MIN_HOOD_DEG))
        val vHoriz  = rpm * EXIT_VEL_M_PER_RPM * cos(Math.toRadians(elevDeg))
        if (vHoriz < 1e-3) return Double.NaN
        return x / vHoriz
    }

    // ─────────────────────────────────────────────────────────────────────────
    // EMA acceleration state  (field frame)
    // vx, vy  — pose-units/s
    // omega   — deg/s  (Pose2d.heading is degrees; we convert to rad/s internally)
    // ─────────────────────────────────────────────────────────────────────────

    private var prevVx    = 0.0; private var prevVy    = 0.0; private var prevOmegaDeg = 0.0
    private var emaAx     = 0.0; private var emaAy     = 0.0; private var emaAlphaRad  = 0.0
    private var lastTimeSec: Double? = null
    private val loopTimer = ElapsedTime().also { it.reset() }

    /** Called automatically by [plan] each tick. */
    fun updateMotion(vx: Double, vy: Double, omegaDeg: Double) {
        val now  = loopTimer.seconds()
        val prev = lastTimeSec
        lastTimeSec = now

        if (prev == null) {
            prevVx = vx; prevVy = vy; prevOmegaDeg = omegaDeg
            return
        }

        val dt = (now - prev).coerceAtLeast(1e-4)
        val a  = ACCEL_ALPHA

        emaAx        = a * ((vx       - prevVx)       / dt) + (1.0 - a) * emaAx
        emaAy        = a * ((vy       - prevVy)       / dt) + (1.0 - a) * emaAy
        // Convert deg/s delta → rad/s² so angular prediction stays in radians
        emaAlphaRad  = a * (Math.toRadians(omegaDeg - prevOmegaDeg) / dt) + (1.0 - a) * emaAlphaRad

        prevVx = vx; prevVy = vy; prevOmegaDeg = omegaDeg
    }

    fun resetMotion() {
        prevVx = 0.0; prevVy = 0.0; prevOmegaDeg = 0.0
        emaAx  = 0.0; emaAy  = 0.0; emaAlphaRad  = 0.0
        lastTimeSec = null
        loopTimer.reset()
    }

    // ─────────────────────────────────────────────────────────────────────────
    // RPM smoother
    // ─────────────────────────────────────────────────────────────────────────

    private val rpmHistory = ArrayDeque<Double>()

    private fun smoothRpm(rpm: Double): Double {
        rpmHistory.addLast(rpm.finiteOrZero())
        while (rpmHistory.size > RPM_SMOOTH_WINDOW.coerceAtLeast(1)) rpmHistory.removeFirst()
        return rpmHistory.average()
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Pose predictor
    // Heading in Pose2d is degrees — prediction done in radians, result back to degrees.
    // ─────────────────────────────────────────────────────────────────────────

    private fun predictPose(
        pose: Pose2d,
        vx: Double, vy: Double, omegaDeg: Double,
        ax: Double, ay: Double, alphaRad: Double,
        horizonSec: Double,
    ): Pose2d {
        val t  = horizonSec.coerceAtLeast(0.0)
        val t2 = t * t
        val omegaRad = Math.toRadians(omegaDeg)
        val predictedHeadingRad = wrapRad(Math.toRadians(pose.heading) + omegaRad * t + 0.5 * alphaRad * t2)
        return Pose2d(
            x       = pose.x + vx * t + 0.5 * ax * t2,
            y       = pose.y + vy * t + 0.5 * ay * t2,
            heading = Math.toDegrees(predictedHeadingRad),
        )
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Virtual goal iteration
    // ─────────────────────────────────────────────────────────────────────────

    private data class IterResult(
        val predictedPose: Pose2d,
        val virtualGoal: Vec2d,
        val totalHorizonSec: Double,
        val possible: Boolean,
    )

    private fun iterateVirtualGoal(
        robotPose: Pose2d,
        vx: Double, vy: Double, omegaDeg: Double,
        ax: Double, ay: Double, alphaRad: Double,
        realGoal: Vec2d,
    ): IterResult {
        if (!SOTM_ENABLED) {
            val predicted = predictPose(robotPose, vx, vy, omegaDeg, ax, ay, alphaRad, SERVO_DELAY_SEC)
            val dist = predicted.distanceTo(realGoal)
            return IterResult(predicted, realGoal, SERVO_DELAY_SEC, isPossible(dist))
        }

        var virtualGoal   = realGoal
        var predictedPose = robotPose
        var totalHorizon  = SERVO_DELAY_SEC

        var _leadX = 0.0
        var _leadY = 0.0
        var _launchX = 0.0
        var _launchY = 0.0

        repeat(SOTM_ITERS) {
            // 1. Pose at current horizon
            predictedPose = predictPose(robotPose, vx, vy, omegaDeg, ax, ay, alphaRad, totalHorizon)

            // 2. Raw distance — lookup handles out-of-range
            val dist = predictedPose.distanceTo(virtualGoal)
            if (!isPossible(dist)) return IterResult(predictedPose, virtualGoal, totalHorizon, false)

            // 3. Lookup + flight time
            val (rpm, hoodNorm) = lookup(dist)
            val tFlight = flightTimeSec(dist, rpm, hoodNorm)
            if (!tFlight.isFinite()) return IterResult(predictedPose, virtualGoal, totalHorizon, false)

            // 4. Update total horizon
            totalHorizon = SERVO_DELAY_SEC + tFlight

            // 5. Robot velocity at the moment of launch (after servo delay)
            val launchVx = vx + ax * SERVO_DELAY_SEC
            val launchVy = vy + ay * SERVO_DELAY_SEC

            _launchX = launchVx
            _launchY = launchVy

            var leadX = launchVx * tFlight * leadMultiplier
            var leadY = launchVy * tFlight * leadMultiplier

            _leadX = leadX
            _leadY = leadY

            val leadMag = hypot(leadX, leadY)
            if (leadMag > MAX_LEAD_POSE_UNITS && leadMag > 1e-6) {
                val s = MAX_LEAD_POSE_UNITS / leadMag
                leadX *= s; leadY *= s
            }

            virtualGoal = Vec2d(realGoal.x - leadX, realGoal.y - leadY)
        }

        tel.addData("[SOTM] lead x",  "%.1f".format(_leadX))
        tel.addData("[SOTM] lead y",  "%.1f".format(_leadY))
        tel.addData("[SOTM] launch x",  "%.1f".format(_launchX))
        tel.addData("[SOTM] launch y",  "%.1f".format(_launchY))

        return IterResult(predictedPose, virtualGoal, totalHorizon, true)
    }

    /**
     * Call every loop tick.
     *
     * @param robotPose   current odom pose — heading in **degrees** (Pose2d convention)
     * @param vx          global x velocity (pose-units/s)
     * @param vy          global y velocity (pose-units/s)
     * @param omegaDeg    angular velocity (deg/s)
     * @param realGoal    fixed goal position in field coordinates
     * @param measuredRpm current flywheel RPM reading
     */
    fun plan(
        robotPose:   Pose2d,
        vx: Double, vy: Double, omegaDeg: Double,
        realGoal:    Vec2d,
        measuredRpm: Double,
    ): ShotCommand {
        updateMotion(vx, vy, omegaDeg)

        val smoothedRpm      = smoothRpm(measuredRpm)
        val ax               = emaAx.finiteOrZero()
        val ay               = emaAy.finiteOrZero()
        val alphaRad         = emaAlphaRad.finiteOrZero()
        val highAccelWarning = hypot(ax, ay) > ACCEL_WARN_POSE_PER_S2

        val result = iterateVirtualGoal(
            robotPose,
            vx.finiteOrZero(), vy.finiteOrZero(), omegaDeg.finiteOrZero(),
            ax, ay, alphaRad,
            realGoal,
        )

        val dist              = result.predictedPose.distanceTo(result.virtualGoal)
        val (targetRpm, hoodNorm) = lookup(dist)
        val possible          = result.possible && isPossible(dist)

        tel?.apply {
            addData("[SOTM] goal x", "%.1f".format(result.virtualGoal.x))
            addData("[SOTM] goal y", "%.1f".format(result.virtualGoal.y))
            addData("[SOTM] dist",     "%.1f pu".format(dist))
            addData("[SOTM] rpm",      "%.0f".format(targetRpm))
            addData("[SOTM] hood",     "%.2f  (%.1f°)".format(hoodNorm, MIN_HOOD_DEG + hoodNorm * (MAX_HOOD_DEG - MIN_HOOD_DEG)))
            addData("[SOTM] horizon",  "%.0f ms".format(result.totalHorizonSec * 1000.0))
            addData("[SOTM] accel x",  "%.1f".format(ax))
            addData("[SOTM] accel y",  "%.1f".format(ay))
            addData("[SOTM] velo x",  "%.1f".format(vx))
            addData("[SOTM] velo y",  "%.1f".format(vy))
            addData("[SOTM] flags",    "highAccel=$highAccelWarning  possible=$possible  extrap=${dist > DATA.lastKey()}")
        }

        return ShotCommand(
            predictedPose     = result.predictedPose,
            virtualGoal       = result.virtualGoal,
            distancePoseUnits = dist,
            targetRpm         = targetRpm,
            hoodNorm          = hoodNorm,
            totalHorizonSec   = result.totalHorizonSec,
            possible          = possible,
            highAccelWarning  = highAccelWarning,
            axPosePerS2       = ax,
            ayPosePerS2       = ay,
            alphaRadPerS2     = alphaRad,
        )
    }

    // Convenience — no motion compensation
    fun lutRpm(dist: Double)      = lookup(dist).first
    fun lutHoodNorm(dist: Double) = lookup(dist).second

    // ─────────────────────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────────────────────

    private fun Double.finiteOrZero() = if (isFinite()) this else 0.0

    private fun wrapRad(r: Double): Double {
        if (!r.isFinite()) return 0.0
        return atan2(sin(r), cos(r))
    }
}