package org.firstinspires.ftc.teamcode.common.subsystem.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.toRadians
import com.millburnx.util.vector.Vec2d
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Limelight
import org.firstinspires.ftc.teamcode.common.util.ZoneAssist
import org.firstinspires.ftc.teamcode.common.util.deadzone
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.opmode.test.pedro.StandaloneRotation
import org.firstinspires.ftc.teamcode.pedro.Constants

@Configurable
class TeleOpDrive(opMode: OpMode, val isRed: Boolean, limelight: Limelight? = null, val intake: Intake? = null) :
    Drive(opMode, limelight) {
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
            gp1.dPad.up.ifPressed { limelight?.localizationState = Limelight.LocalizationState.NONE }

            follower.update()
            drawRobot()

            tel.addData("inZone", inZone)
            tel.addData("useZoneAssist", useZoneAssist)

            if (!isTeleopDrive) return@with

            gp1.b.ifPressed { useGateAssist = !useGateAssist }
            gp1.a.ifPressed { useZoneAssist = !useZoneAssist }

            if (inZone) useZoneAssist = false
            if ((intake?.targetPower ?: 0.0) < 0.0 || useZoneAssist) useGateAssist = false

            val zoneAssist = if (useZoneAssist) {
                ZoneAssist.calculateRelativeAssist(pose)
            } else {
                Vec2d()
            }

            follower.setTeleOpDrive(
                -gp1.current.leftJoyStick.y.deadzone(minPower) + zoneAssist.x,
                -gp1.current.leftJoyStick.x.deadzone(minPower) + zoneAssist.y,
                -gp1.current.rightJoyStick.x + if (useGateAssist) gateAssist() else 0.0,
            )
        }
    }

    companion object {
        @JvmField
        var gateAssistHeading = 35.0

        @JvmField
        var minPower = 0.3
    }
}