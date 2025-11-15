package org.firstinspires.ftc.teamcode.opmode.auton

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
class BlueAuton: CloseAuton(isRed = false)

@Autonomous
class RedAuton: CloseAuton(isRed = true)

open class CloseAuton(val isRed: Boolean) : OpMode() {
    override fun run() {
        val intake = Intake(this, isTeleop = false)
        val flyWheel = FlyWheel(this, isTeleop = false)
        val uppies = Uppies(this, { flyWheel.state }, isTeleop = false)
        val pedro = Pedro(this, Pose2d(p(Vec2d(26.0,129.0)), p(145.0)), isTeleop = false)

        uppies.nextState()

        val shootBallOne = PedroPath(
            pedro.follower, Line(
                p(Vec2d(26, 129)), p(Vec2d(48, 104))
            ), LinearHeading(p(145.0), p(140.0))
        )

        val preRowOne = PedroPath(
            pedro.follower, Line(
                p(Vec2d(48, 104)), p(Vec2d(48, 92))
            ),
            LinearHeading(p(135.0), p(5.0))
        )

        val intakeRowOne = PedroPath(
            pedro.follower, Line(
                p(Vec2d(48, 94)), p(Vec2d(25, 94))
            ),
            LinearHeading(p(0.0), p(5.0))
        )

        val shootRowOne = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(26, 94)),
                p(Vec2d(28, 96)),
                p(Vec2d(32, 96)),
                p(Vec2d(48, 110))
            ),
            LinearHeading(p(0.0), p(140.0))
        )

        val preRowTwo = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(48, 96)),
                p(Vec2d(60, 76)),
                p(Vec2d(54, 76)),
                p(Vec2d(48, 78)),
            ), LinearHeading(p(135.0), p(0.0))
        )

        val intakeRowTwo = PedroPath(
            pedro.follower, Line(
                p(Vec2d(48, 74)), p(Vec2d(16, 70))
            ),
            LinearHeading(p(5.0), p(10.0))
        )

        val shootRowTwo = PedroPath(
            pedro.follower, CubicBezier(
                p(Vec2d(16, 70)),
                p(Vec2d(48, 70)),
                p(Vec2d(48, 70)),
                p(Vec2d(48, 120)),
            ),
            LinearHeading(p(0.0), p(145.0))
        )

        scheduler.schedule(Sequential {
            Command { WaitFor { isStarted } }
            +FollowPath(pedro.follower, shootBallOne) { opModeIsActive() }
            Command { SleepFor { 500 } }
            +AutoFire(this@CloseAuton, null, intake, flyWheel, uppies, isTeleop = false)
            +FollowPath(pedro.follower, preRowOne) { opModeIsActive() }
            Command { intake.power = 1.0 }
            +FollowPath(pedro.follower, intakeRowOne) { opModeIsActive() }
            Command { SleepFor { 250 }; uppies.nextState(); SleepFor { 500 } }f
            +FollowPath(pedro.follower, shootRowOne) { opModeIsActive() }
            Command { SleepFor { 500 } }
            +AutoFire(this@CloseAuton, null, intake, flyWheel, uppies, isTeleop = false)
            +FollowPath(pedro.follower, preRowTwo) { opModeIsActive() }
            Command { intake.power = 1.0 }
            +FollowPath(pedro.follower, intakeRowTwo) { opModeIsActive() }
            Command { SleepFor { 250 }; uppies.nextState(); SleepFor { 250 } }
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
}