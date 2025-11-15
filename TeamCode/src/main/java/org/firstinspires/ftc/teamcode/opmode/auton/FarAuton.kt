package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.FollowPath
import com.millburnx.cmdxpedro.paths.PedroPath
import com.millburnx.cmdxpedro.paths.heading.LinearHeading
import com.millburnx.cmdxpedro.paths.heading.TangentialHeading
import com.millburnx.cmdxpedro.paths.path.CubicBezier
import com.millburnx.cmdxpedro.paths.path.Line
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.commands.AutoFire
import org.firstinspires.ftc.teamcode.common.fieldMirror
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Pedro
import org.firstinspires.ftc.teamcode.common.subsystem.Uppies
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Autonomous(preselectTeleOp = "Teleop")
class BlueFarAuton : FarAuton(isRed = false)

@Autonomous(preselectTeleOp = "Teleop")
class RedFarAuton : FarAuton(isRed = true)

@Configurable
open class FarAuton(val isRed: Boolean) : OpMode() {
    override fun run() {
        val intake = Intake(this, isTeleop = false)
        val flyWheel = FlyWheel(this, isTeleop = false)
        val uppies = Uppies(this, { flyWheel.state }, isTeleop = false)
        val pedro = Pedro(this, Pose2d(p(Vec2d(56.0, 8.0)), p(90.0)), isTeleop = false)

        uppies.nextState()

        val shootPreload = PedroPath(
            pedro.follower, Line(
                p(Vec2d(56.0, 8.0)),
                p(Vec2d(shootX0, shootY0)),
            ), LinearHeading(p(90.0), p(shootH0))
        )

        val preRowOne = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(shootX0, shootY0)),
                p(Vec2d(shootX0, postRowX1)),
                p(Vec2d(shootX0, postRowX1)),
                p(Vec2d(preRowX, postRowX1)),
            ), LinearHeading(p(shootH0), p(0.0))
        )

        val intakeRowOne = PedroPath(
            pedro.follower, Line(
                p(Vec2d(preRowX, rowY1)),
                p(Vec2d(postRowX1, rowY1)),
            ), TangentialHeading(reverse = true)
        )

        val shootRowOne = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(postRowX1, rowY1)),
                p(Vec2d(postRowX1 + 24.0, rowY1)),
                p(Vec2d(shootX1 - 24.0, shootY1)),
                p(Vec2d(shootX1, rowY1)),
            ), LinearHeading(p(0.0), p(shootH1))
        )

        val preRowTwo = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(shootX1, rowY1)),
                p(Vec2d(shootX1, rowY2 + 2.0)),
                p(Vec2d(shootX1, rowY2 + 2.0)),
                p(Vec2d(preRowX, rowY2 + 2.0)),
            ), LinearHeading(p(shootH1), p(0.0))
        )

        val intakeRowTwo = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(preRowX, rowY2 + 2.0)),
                p(Vec2d(preRowX - 24.0, rowY2 + 2.0)),
                p(Vec2d(postRowX2 + 24.0, rowY2 - 2.0)),
                p(Vec2d(postRowX2, rowY2 - 2.0)),
            ), TangentialHeading(reverse = true)
        )

        val shootRowTwo = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(postRowX2, rowY2)),
                p(Vec2d(48.0, rowY2)),
                p(Vec2d(shootX2, shootY2)),
                p(Vec2d(shootX2, shootY2)),
            ), LinearHeading(p(0.0), p(shootH2))
        )

        scheduler.schedule(Sequential {
            Command { WaitFor { isStarted } }
            +FollowPath(pedro.follower, shootPreload) { opModeIsActive() }
            Command { SleepFor { 500 } }
            +AutoFire(this@FarAuton, null, intake, flyWheel, uppies, isTeleop = false)


            +FollowPath(pedro.follower, preRowOne) { opModeIsActive() }
            Command { intake.power = 1.0 }
            +FollowPath(pedro.follower, intakeRowOne) { opModeIsActive() }
            Command { SleepFor { 125 }; uppies.nextState(); SleepFor { 375 } }
            +FollowPath(pedro.follower, shootRowOne) { opModeIsActive() }
            Command { SleepFor { 500 } }
            +AutoFire(this@FarAuton, null, intake, flyWheel, uppies, isTeleop = false)

            +FollowPath(pedro.follower, preRowTwo) { opModeIsActive() }
            Command { intake.power = 1.0 }
            +FollowPath(pedro.follower, intakeRowTwo) { opModeIsActive() }
            Command { SleepFor { 125 }; uppies.nextState(); SleepFor { 625 } }
            +FollowPath(pedro.follower, shootRowTwo) { opModeIsActive() }
            Command { SleepFor { 500 } }
            +AutoFire(this@FarAuton, null, intake, flyWheel, uppies, isTeleop = false)
        })
    }

    fun p(vec2d: Vec2d): Vec2d {
        return if (isRed) vec2d.fieldMirror() else vec2d
    }

    fun p(double: Double): Double {
        return if (isRed) double.fieldMirror() else double
    }

    companion object {
        @JvmField
        var preRowX = 44.0

        @JvmField
        var postRowX1 = 9.0

        @JvmField
        var postRowX2 = 9.0

        @JvmField
        var rowY1 = 36.0

        @JvmField
        var rowY2 = 60.0

        // shooting

        @JvmField
        var shootX0 = 56.0

        @JvmField
        var shootX1 = 56.0

        @JvmField
        var shootX2 = 56.0

        @JvmField
        var shootY0 = 12.0

        @JvmField
        var shootY1 = 12.0

        @JvmField
        var shootY2 = 12.0

        @JvmField
        var shootH0 = 111.0

        @JvmField
        var shootH1 = 111.0

        @JvmField
        var shootH2 = 111.0
    }
}