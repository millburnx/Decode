package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.PanelsField
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.millburnx.util.toRadians
import com.millburnx.util.vector.Vec2d
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.common.hardware.fromPedro
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.common.util.ZoneAssist
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.opmode.test.pedro.StandaloneRotation
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.abs

@Configurable
open class Drive(val opMode: OpMode, limelight: Limelight? = null) : Subsystem("Drive") {
    open val follower = Constants.createManualFollower(opMode.hardwareMap)

    var pose
        get() = Pose2d.fromPedro(follower.pose ?: Pose(0.0, 0.0, 0.0))
        set(value) {
            follower.pose = value.toPedro()
        }

    val velocity
        get() = Pose2d(follower.velocity.xComponent, follower.velocity.yComponent, follower.angularVelocity)

    val canvas = PanelsField.field

    override val run: suspend Command.() -> Unit = {
        canvas.setOffsets(PanelsField.presets.PEDRO_PATHING)
        this@Drive.init(this)
        OpModeLoop(opMode) {
            this@Drive.loop(this)
        }
    }

    open val init: suspend Command.() -> Unit = {
        follower.update()
        while (!opMode.isStopRequested && !opMode.isStarted) {
            drawRobot()
            sync()
        }
    }

    open val loop: suspend Command.() -> Unit = {
        follower.update()
        drawRobot()
//        println("end pose ${follower.currentPath?.endPose()}")
    }

    fun drawRobot(pose: Pose2d = this.pose, name: String = "pose", color: String = "white") {
        canvas.moveCursor(pose.x, pose.y)
        canvas.setStyle(fill = "none", outline = color, width = 1.5)
        canvas.circle(8.0)
        val lookPos = pose + Vec2d(1.0, 0.0).rotate(pose.radians) * 8.0
        canvas.line(lookPos.x, lookPos.y)

        if (useTelemetry) {
            opMode.tel.addData("$name.x", pose.x)
            opMode.tel.addData("$name.y", pose.y)
            opMode.tel.addData("$name.h", pose.heading)
        }
    }

    companion object {
        @JvmField
        var useTelemetry = false
    }
}

@Configurable
class TeleOpDrive(opMode: OpMode, val isRed: Boolean, limelight: Limelight? = null) : Drive(opMode, limelight) {
    override val follower = Constants.createManualFusionFollower(opMode.hardwareMap, { opMode.deltaTime }, limelight)

    val headingLockController = StandaloneRotation()

    var useGateAssist = false

    var useZoneAssist = false

    override val init: suspend Command.() -> Unit = {
        follower.update()
        follower.startTeleopDrive(false)

        while (!opMode.isStopRequested && !opMode.isStarted) {
            drawRobot()
            sync()
        }
    }

    fun gateAssist(): Double {
        val target = gateAssistHeading.mirror(!isRed)
        return headingLockController.calc(pose.radians, target.toRadians())
    }

    val inZonePartial: Boolean
        get() = ZoneAssist.inZonePartial(pose.position)

    val inZoneClosePartial: Boolean
        get() = ZoneAssist.inZonePartial(pose.position, useFar = false)

    val inZoneFarPartial: Boolean
        get() = ZoneAssist.inZonePartial(pose.position, useClose = false)

    val inZone: Boolean
        get() = ZoneAssist.inZone(pose.position)

    var isTeleopDrive = true
        set(value) {
            if (!field && value) {
                follower.breakFollowing()
                follower.startTeleopDrive(false)
            }
            field = value
        }

    override val loop: suspend Command.() -> Unit = {
        with(opMode) {
            follower.update()
            drawRobot()

            tel.addData("inZone", inZone)
            tel.addData("useZoneAssist", useZoneAssist)

            if (isTeleopDrive) {
                if (!gp1.prev.b && gp1.current.b) useGateAssist = !useGateAssist
                if (!gp1.prev.a && gp1.current.a) useZoneAssist = !useZoneAssist
                if (!useGateAssist) {
                    val zoneAssist = if (useZoneAssist) {
                        if (useFieldCentric) {
                            ZoneAssist.calculateAssist(pose.position)
                        } else {
                            ZoneAssist.calculateRelativeAssist(
                                pose
                            )
                        }
                    } else {
                        Vec2d()
                    }

                    if (inZone) useZoneAssist = false // disable on enter

                    var forward = -gp1.current.leftJoyStick.y
                    if (abs(forward) < minPower) forward = 0.0

                    var strafe = -gp1.current.leftJoyStick.x
                    if (abs(strafe) < minPower) strafe = 0.0

                    follower.setTeleOpDrive(
                        forward + zoneAssist.x,
                        strafe + zoneAssist.y,
                        -gp1.current.rightJoyStick.x,
                        !useFieldCentric
                    )
                } else {
                    follower.setTeleOpDrive(
                        -gp1.current.leftJoyStick.y,
                        -gp1.current.leftJoyStick.x,
                        -gp1.current.rightJoyStick.x + gateAssist(),
                        !useFieldCentric
                    )
                }
            }
        }
    }

    companion object {
        @JvmField
        var gateAssistHeading = 35.0

        @JvmField
        var useFieldCentric = false

        @JvmField
        var minPower = 0.3
    }
}