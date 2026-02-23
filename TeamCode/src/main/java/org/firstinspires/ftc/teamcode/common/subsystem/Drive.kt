package org.firstinspires.ftc.teamcode.common.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.util.Pose2d
import org.firstinspires.ftc.teamcode.common.hardware.fromPedro
import org.firstinspires.ftc.teamcode.common.hardware.normalizeDegrees
import org.firstinspires.ftc.teamcode.common.hardware.toPedro
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants

@Configurable
open class Drive(val opMode: OpMode, limelight: Limelight? = null) : Subsystem("Drive") {
    val follower = Constants.createManualFusionFollower(opMode.hardwareMap, { opMode.deltaTime }, limelight)

    var pose
        get() = Pose2d.fromPedro(follower.pose)
        set(value) {
            follower.pose = value.toPedro()
        }

    val velocity
        get() = Pose2d(follower.velocity.xComponent, follower.velocity.yComponent, follower.angularVelocity)

    override val run: suspend Command.() -> Unit = {
        this@Drive.init(this)
        OpModeLoop(opMode) {
            this@Drive.loop(this)
        }
    }

    open val init: suspend Command.() -> Unit = {
        follower.update()
    }

    open val loop: suspend Command.() -> Unit = {
        follower.update()
    }

    companion object {

    }
}

class TeleOpDrive(opMode: OpMode, limelight: Limelight? = null) : Drive(opMode, limelight) {
    var useGateAssist = false

    override val init: suspend Command.() -> Unit = {
        follower.update()
        follower.startTeleopDrive(false)
    }

    fun gateAssist(): Double {
        val heading = follower.pose.heading
        val diff = normalizeDegrees(heading - gateAssistHeading)
        return diff * gateAssistPower
    }

    override val loop: suspend Command.() -> Unit = {
        with(opMode) {
            follower.update()
            if (!follower.isTeleopDrive && !follower.isBusy && !follower.isTurning) {
                follower.startTeleopDrive(false)
            }
            if (follower.isTeleopDrive) {
                if (!useGateAssist) {
                    follower.setTeleOpDrive(
                        gp1.current.leftJoyStick.y,
                        gp1.current.leftJoyStick.x,
                        gp1.current.rightJoyStick.x,
                        !useFieldCentric
                    )
                } else {
                    follower.setTeleOpDrive(
                        gp1.current.leftJoyStick.y,
                        gp1.current.leftJoyStick.x,
                        gp1.current.rightJoyStick.x + gateAssist(),
                        false
                    )
                }
            }
        }
    }

    companion object {
        @JvmField
        var gateAssistHeading = 45.0

        @JvmField
        var gateAssistPower = 0.025

        @JvmField
        var useFieldCentric = false
    }
}