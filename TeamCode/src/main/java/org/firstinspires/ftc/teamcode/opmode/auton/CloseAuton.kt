package org.firstinspires.ftc.teamcode.opmode.auton

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.FollowPath
import com.millburnx.cmdxpedro.paths.PedroPath
import com.millburnx.cmdxpedro.paths.heading.LinearHeading
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

@Autonomous
class BlueAuton : CloseAuton(isRed = false)

@Autonomous
class RedAuton : CloseAuton(isRed = true)

@Configurable
open class CloseAuton(val isRed: Boolean) : OpMode() {
    override fun run() {
        val intake = Intake(this, isTeleop = false)
        val flyWheel = FlyWheel(this, isTeleop = false)
        val uppies = Uppies(this, { flyWheel.state }, isTeleop = false)
        val pedro = Pedro(this, Pose2d(p(Vec2d(15.0, 111.0)), p(90.0)), isTeleop = false)

        uppies.nextState()

        val shootBallOne = PedroPath(
            pedro.follower, Line(
                p(Vec2d(15, 111)), p(Vec2d(32, shootY0))
            ), LinearHeading(p(90.0), p(135.0))
        )

        val preRowOne = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(32, shootY0)),
                p(Vec2d(48, shootY0)),
                p(Vec2d(preRowX + 12, rowY1)),
                p(Vec2d(preRowX, rowY1))
            ),
            LinearHeading(p(135.0), p(0.0))
        )

        val intakeRowOne = PedroPath(
            pedro.follower, Line(
                p(Vec2d(preRowX, rowY1)), p(Vec2d(16, rowY1))
            ),
            LinearHeading(p(0.0), p(0.0))
        )

        val shootRowOne = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(16, rowY1)),
                p(Vec2d(48, rowY1)),
                p(Vec2d(48, rowY1)),
                p(Vec2d(48, shootY1))
            ),
            LinearHeading(p(0.0), p(135.0))
        )

        val preRowTwo = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(48, shootY1)),
                p(Vec2d(60, shootY1-12)),
                p(Vec2d(60, rowY2 + 2)),
                p(Vec2d(preRowX, rowY2 + 2)),
            ), LinearHeading(p(135.0), p(5.0))
        )

        val intakeRowTwo = PedroPath(
            pedro.follower, Line(
                p(Vec2d(preRowX, rowY2 + 2)), p(Vec2d(9, rowY2 - 2))
            ),
            LinearHeading(p(5.0), p(0.0))
        )

        val shootRowTwo = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(9, rowY2 - 2)),
                p(Vec2d(24, rowY2 - 2)),
                p(Vec2d(48, 72)),
                p(Vec2d(48, shootY2)),
            ),
            LinearHeading(p(0.0), p(135.0))
        )

        scheduler.schedule(Sequential {
            Command { WaitFor { isStarted } }
            +FollowPath(pedro.follower, shootBallOne) { opModeIsActive() }
            Command { SleepFor { 500 } }
            +AutoFire(this@CloseAuton, null, intake, flyWheel, uppies, isTeleop = false)

            +FollowPath(pedro.follower, preRowOne) { opModeIsActive() }
            Command { intake.power = 1.0 }
            +FollowPath(pedro.follower, intakeRowOne) { opModeIsActive() }
            Command { SleepFor { 125 }; uppies.nextState(); SleepFor { 625 } }
            +FollowPath(pedro.follower, shootRowOne) { opModeIsActive() }
            Command { SleepFor { 500 } }
            +AutoFire(this@CloseAuton, null, intake, flyWheel, uppies, isTeleop = false)

            +FollowPath(pedro.follower, preRowTwo) { opModeIsActive() }
            Command { intake.power = 1.0 }
            +FollowPath(pedro.follower, intakeRowTwo) { opModeIsActive() }
            Command { SleepFor { 125 }; uppies.nextState(); SleepFor { 375 } }
            +FollowPath(pedro.follower, shootRowTwo) { opModeIsActive() }
            Command { SleepFor { 500 } }
            +AutoFire(this@CloseAuton, null, intake, flyWheel, uppies, isTeleop = false)
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
        var preRowX = 48.0

        @JvmField
        var rowY1 = 84.0

        @JvmField
        var rowY2 = 60.0

        @JvmField
        var shootY0 = 111

        @JvmField
        var shootY1 = 96

        @JvmField
        var shootY2 = 96
    }
}