package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.PanelsField
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.millburnx.util.vector.Vec2d
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.common.hardware.fromPedro
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.common.util.ZoneAssist
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants

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
        println("end pose ${follower.currentPath?.endPose()}")
    }

    fun drawRobot(pose: Pose2d = this.pose, name: String = "pose", color: String = "white") {
        canvas.moveCursor(pose.x, pose.y)
        canvas.setStyle(fill = "none", outline = color, width = 1.5)
        canvas.circle(8.0)
        val lookPos = pose + Vec2d(1.0, 0.0).rotate(pose.radians) * 8.0
        canvas.line(lookPos.x, lookPos.y)

        opMode.tel.addData("$name.x", pose.x)
        opMode.tel.addData("$name.y", pose.y)
        opMode.tel.addData("$name.h", pose.heading)
    }

    companion object {

    }
}

@Configurable
class TeleOpDrive(opMode: OpMode, val isRed: Boolean, limelight: Limelight? = null) : Drive(opMode, limelight) {
    override val follower = Constants.createManualFusionFollower(opMode.hardwareMap, { opMode.deltaTime }, limelight)

    var useGateAssist = false

    override val init: suspend Command.() -> Unit = {
        follower.update()
        follower.startTeleopDrive(false)

        while (!opMode.isStopRequested && !opMode.isStarted) {
            drawRobot()
            sync()
        }
    }

    fun gateAssist(): Double {
        val heading = pose.heading
        val diff = normalizeDegrees(heading - gateAssistHeading.mirror(!isRed))
        opMode.tel.addData("gateassist", diff)
        return diff * gateAssistPower
    }

    val inZone: Boolean
        get() {
            return ZoneAssist.farZone.contains(pose.position) || ZoneAssist.closeZone.contains(pose.position)
        }

    override val loop: suspend Command.() -> Unit = {
        with(opMode) {
            follower.update()
            drawRobot()

            if (!follower.isTeleopDrive && !follower.isBusy && !follower.isTurning) {
                follower.startTeleopDrive(false)
            }
            if (follower.isTeleopDrive) {
                if (!gp1.prev.b && gp1.current.b) useGateAssist = !useGateAssist
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

                    follower.setTeleOpDrive(
                        -gp1.current.leftJoyStick.y + zoneAssist.x,
                        -gp1.current.leftJoyStick.x + zoneAssist.y,
                        -gp1.current.rightJoyStick.x,
                        !useFieldCentric
                    )
                } else {
                    follower.setTeleOpDrive(
                        -gp1.current.leftJoyStick.y,
                        -gp1.current.leftJoyStick.x,
                        -(gp1.current.rightJoyStick.x + gateAssist()),
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
        var gateAssistPower = 0.02

        @JvmField
        var useFieldCentric = false

        @JvmField
        var useZoneAssist = false
    }
}