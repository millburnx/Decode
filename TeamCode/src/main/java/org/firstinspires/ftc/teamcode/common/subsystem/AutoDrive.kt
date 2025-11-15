package org.firstinspires.ftc.teamcode.common.subsystem

import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.Pose2d
import com.millburnx.cmdxpedro.util.WaitFor
import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d
import com.millburnx.cmdxpedro.util.toDegrees
import com.pedropathing.geometry.BezierLine
import org.firstinspires.ftc.teamcode.common.fieldMirror
import org.firstinspires.ftc.teamcode.common.normalizeRadians
import org.firstinspires.ftc.teamcode.common.toPedro
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.sign


class AutoDrive(
    opMode: OpMode, pedro: Pedro, autoTargeting: AutoTargeting
) : Subsystem("Auto Drive") {
    override val run: suspend Command.() -> Unit = Command@{

        with(opMode) {
            WaitFor { isStarted }

            while (!isStopRequested) {
                val target = autoTargeting.target
                val targetATag = autoTargeting.targetATag

                if (pedro.isLocked && !pedro.follower.isBusy && !pedro.follower.isTurning) {
                    pedro.isLocked = false
                    pedro.follower.startTeleopDrive(false)
                }

                if (gp1.current.dPad.up && !gp1.current.dPad.down) {
                    if (target == null || targetATag == null || pedro.isLocked) {
                        sync()
                        continue
                    }

                    val targetPoint = getTargetPoint(pedro, target, targetATag)

                    if (targetPoint != null) {
                        pedro.isLocked = true
                        val path = pedro.follower.pathBuilder()
                            .addPath(
                                BezierLine(
                                    pedro.pose.toPedro(),
                                    targetPoint.toPedro(),
                                )
                            )
                            .setLinearHeadingInterpolation(pedro.pose.radians, target.radians)
                            .build()
                        pedro.follower.followPath(path)
                        println("TARGET POINT $targetPoint" +
                                "")
                    }
                }

                sync()
            }
        }
    }

    fun getTargetPoint(pedro: Pedro, target: Pose2d, targetATag: AprilTagDetection): Pose2d? {
        val tagPositionGlobal: Pose2d =
            Pose2d(16.37007874, 130.3464567, 145.0).let {
                if (targetATag.id == Apriltags.RED_ID) it.fieldMirror() else it
            }

        val triangle = Triangle(
            Vec2d(0.0, 144.0),
            Vec2d(72.0, 72.0),
            Vec2d(144.0, 144.0),
        ).map {
            globalToRobotSpace(tagPositionGlobal, target, it)
        }

        val withinTriangle = triangle.contains(pedro.pose.position)

        if (withinTriangle) return null

        val blueLine = LineSegment(
            Vec2d(0.0, 144.0) + Vec2d(24.0, -24.0),
            Vec2d(72.0, 72.0),
        ).map {
            globalToRobotSpace(tagPositionGlobal, target, it)
        }

        val redLine = LineSegment(
            Vec2d(0.0, 144.0) + Vec2d(-24.0, -24.0),
            Vec2d(72.0, 72.0),
        ).map {
            globalToRobotSpace(tagPositionGlobal, target, it)
        }

        val closestBlue = blueLine.project(pedro.pose.position)
        val closestRed = redLine.project(pedro.pose.position)
        val closestPoint = if (pedro.pose.distanceTo(closestBlue) <= pedro.pose.distanceTo(closestRed)) {
            closestBlue
        } else {
            closestRed
        }

        println("DATATA $triangle $withinTriangle $blueLine $redLine $closestBlue $closestRed $closestPoint")

        val angle: Double = closestPoint.angleTo(target.position)
        return Pose2d(closestPoint, angle)
    }

    fun globalToRobotSpace(tagGlobal: Pose2d, tagRobotSpace: Pose2d, point: Vec2d): Vec2d {
        val robotSpaceHeadingError = normalizeRadians(tagGlobal.radians - tagRobotSpace.radians)

        println("tagPositionGlobal $tagGlobal robotSpaceHeadingError ${robotSpaceHeadingError.toDegrees()}")

        val tagRelative = point - tagGlobal.position
        val robotSpaceTagRelative = tagRelative.rotate(robotSpaceHeadingError)
        val robotSpace = robotSpaceTagRelative + tagRobotSpace.position
        return robotSpace
    }
}

data class Triangle(val p0: Vec2d, val p1: Vec2d, val p2: Vec2d) {
    fun map(mapper: (Vec2d) -> Vec2d): Triangle {
        return Triangle(mapper(p0), mapper(p1), mapper(p2))
    }

    fun contains(point: Vec2d): Boolean {
        fun halfPlaneSign(p: Vec2d, a: Vec2d, b: Vec2d): Double {
            val A = (p.x - b.x) * (a.y - b.y)
            val B = (a.x - b.x) * (p.y - b.y)
            return A - B
        }

        val sign01 = halfPlaneSign(point, p0, p1)
        val sign12 = halfPlaneSign(point, p1, p2)
        val sign20 = halfPlaneSign(point, p2, p0)

        return sign(sign01) == sign(sign12) && sign(sign20) == sign(sign12)
    }
}

data class LineSegment(val p0: Vec2d, val p1: Vec2d) {
    fun map(mapper: (Vec2d) -> Vec2d): LineSegment {
        return LineSegment(mapper(p0), mapper(p1))
    }

    fun project(point: Vec2d): Vec2d {
        val v01 = p1 - p0
        val v0p = point - p0
        val t = (v01.dot(v0p) / v01.dot(v01)).coerceIn(0.0, 1.0)

        return p0 + v01 * t
    }
}