package org.firstinspires.ftc.teamcode.pedro

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.util.Pose2d
import com.pedropathing.ftc.FollowerBuilder
import com.pedropathing.geometry.Pose
import com.pedropathing.localization.Localizer
import com.pedropathing.math.Vector
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.common.GlobalStore
import org.firstinspires.ftc.teamcode.common.hardware.fromFTC
import org.firstinspires.ftc.teamcode.common.hardware.fromPedro
import org.firstinspires.ftc.teamcode.common.hardware.toFTC
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.common.subsystem.Limelight
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min

@Configurable
class FusionLocalizer(hardwareMap: HardwareMap, val deltaTime: () -> Double, val limelight: Limelight? = null) :
    Localizer {
    val pinpoint = (hardwareMap.get("pinpoint") as GoBildaPinpointDriver).apply {
        setOffsets(PinpointSettings.forwardOffset, PinpointSettings.strafeOffset, DistanceUnit.MM)
        setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)

        val reversedPod = GoBildaPinpointDriver.EncoderDirection.REVERSED
        val forwardPod = GoBildaPinpointDriver.EncoderDirection.FORWARD
        setEncoderDirections(
            if (PinpointSettings.forwardReversed) reversedPod else forwardPod,
            if (PinpointSettings.strafeReversed) reversedPod else forwardPod
        )

        resetPosAndIMU()
    }

    init {
        limelight?.getPose = { _pose }
        limelight?.setPose = { setPose(it) }
//        while (pinpoint.deviceStatus != GoBildaPinpointDriver.DeviceStatus.READY) {
//            Thread.yield()
//        }
        Thread.sleep(300)
    }

    var _pose: Pose2d = Pose2d()
    var _drift: Pose2d = Pose2d()
    var _velocity: Pose2d = Pose2d()

    var _lastLLTimestamp = 0L

    val kfX = DriftKalmanFilter { kfConfig }
    val kfY = DriftKalmanFilter { kfConfig }

    private val lock = ReentrantLock()

    override fun getPose() = lock.withLock { (_pose - _drift).toPedro() } ?: Pose()
    override fun getVelocity() = lock.withLock { _velocity.toPedro() } ?: Pose()
    override fun getVelocityVector() = lock.withLock { _velocity.toPedro().asVector ?: Vector() } ?: Vector()

    override fun setStartPose(p0: Pose) = setPose(p0) // low-key cannot be bothered to compensate, just don't update?

    fun setPose(p0: Pose2d) {
        pinpoint.position = p0.toFTC()
        _pose = p0
        _drift = Pose2d()

        kfX.reset()
        kfY.reset()
    }

    override fun setPose(p0: Pose) = setPose(Pose2d.fromPedro(p0))

    override fun update() {
        lock.withLock {
            pinpoint.update()
            val ppPose = pinpoint.position
            if (ppPose != null) {
                _pose = Pose2d.fromFTC(ppPose)
            }

            kfX.predict(deltaTime())
            kfY.predict(deltaTime())

            if (limelight != null && GlobalStore.useKF) {
                val llPose = limelight.pose
                if (llPose != null && llPose.second != _lastLLTimestamp) {
                    _lastLLTimestamp = llPose.second
                    kfX.update(_pose.x, llPose.first.x)
                    kfY.update(_pose.y, llPose.first.y)
                }
            }

            _drift = Pose2d(kfX.drift, kfY.drift, 0.0)

            _velocity = Pose2d(
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)
            )
        }
    }

    override fun getTotalHeading() = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)

    override fun getForwardMultiplier() = pinpoint.encoderY.toDouble()
    override fun getLateralMultiplier() = pinpoint.encoderX.toDouble()
    override fun getTurningMultiplier() = pinpoint.yawScalar.toDouble()

    override fun resetIMU() = pinpoint.resetPosAndIMU()
    override fun getIMUHeading(): Double = Double.NaN
    override fun isNAN(): Boolean = pose.x.isNaN() || pose.y.isNaN() || pose.heading.isNaN()

    companion object {
        @JvmField
        var kfConfig = DriftKalmanFilter.Config(
            startingUncertainty = 1.0,
            processNoise = 0.0001, // q
            measurementNoise = 12.0, // r
            maxGain = 0.01,
            maxDist = 12.0,
            minUncertainty = 1e-6
        )
    }
}

fun FollowerBuilder.fusionLocalizer(
    hardwareMap: HardwareMap,
    deltaTime: () -> Double,
    limelight: Limelight? = null
): FollowerBuilder {
    return setLocalizer(FusionLocalizer(hardwareMap, deltaTime, limelight))
}

@Configurable
class DriftKalmanFilter(val config: () -> Config) {
    var drift = 0.0
    var uncertainty = config().startingUncertainty

    fun reset() {
        drift = 0.0
        uncertainty = config().startingUncertainty
    }

    fun predict(dt: Double) {
        uncertainty += config().processNoise * dt
    }

    fun update(pinpoint: Double, limelight: Double) {
        val rawDrift = pinpoint - limelight
        if (abs(rawDrift - drift) > config().maxDist) return

        val rawGain = uncertainty / (uncertainty + config().measurementNoise)
        val gain = min(config().maxGain * (1 - exp(-rawGain / config().maxGain)), config().maxGain)

        drift += gain * (rawDrift - drift)

        uncertainty = max(uncertainty * (1 - gain), config().minUncertainty)
    }

    data class Config(
        var startingUncertainty: Double,
        var processNoise: Double, // q
        var measurementNoise: Double, // r
        var maxGain: Double,
        var maxDist: Double,
        var minUncertainty: Double
    )
}

@Configurable
object PinpointSettings {
    @JvmField
    var forwardOffset = -150.0

    @JvmField
    var strafeOffset = -138.0

    @JvmField
    var forwardReversed = false

    @JvmField
    var strafeReversed = false
}