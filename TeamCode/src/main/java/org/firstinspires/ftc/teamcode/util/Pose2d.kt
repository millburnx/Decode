package org.firstinspires.ftc.teamcode.util

import com.pedropathing.geometry.Pose
import kotlin.math.absoluteValue
import kotlin.math.sign

data class Pose2d(
    val position: Vec2d = Vec2d(),
    val heading: Double = 0.0,
) {
    constructor(x: Double, y: Double, heading: Double) : this(Vec2d(x, y), heading)
    constructor(x: Double, y: Double) : this(Vec2d(x,y), 0.0)
    constructor(values: Array<Double>) : this(values[0], values[1], values[2])

    val x: Double
        get() = position.x
    val y: Double
        get() = position.y
    val degrees: Double
        get() = heading
    val radians: Double
        get() = Math.toRadians(heading)
    val rotation: Double
        get() = heading

    companion object {
        fun fromRadians(
            position: Vec2d,
            radians: Double,
        ): Pose2d = Pose2d(position, Math.toDegrees(radians))

        fun fromRadians(
            x: Double,
            y: Double,
            radians: Double,
        ): Pose2d = Pose2d(x, y, Math.toDegrees(radians))

//        fun fromRR(pose: com.acmerobotics.roadrunner.geometry.Pose2d): Pose2d = Pose2d(pose.x, pose.y, Math.toDegrees(pose.heading))

        fun fromRR(pose: Pose2d): Pose2d = Pose2d(pose.x, pose.y, Math.toDegrees(pose.heading))
    }

    fun toRR(): Pose2d = Pose2d(position.y, -position.x, heading)

//    fun toRR(): com.acmerobotics.roadrunner.geometry.Pose2d =
//        com.acmerobotics.roadrunner.geometry
//            .Pose2d(position.y, -position.x, Math.toRadians(heading))
//
//    fun toRawRR(): com.acmerobotics.roadrunner.geometry.Pose2d =
//        com.acmerobotics.roadrunner.geometry
//            .Pose2d(position.x, position.y, Math.toRadians(heading))

    operator fun unaryMinus(): Pose2d = Pose2d(-position, -heading)

    fun flipPos(): Pose2d = Pose2d(-position, heading)

    fun abs(): Pose2d = Pose2d(position.abs(), heading.absoluteValue)

    operator fun plus(other: Pose2d): Pose2d = Pose2d(position + other.position, normalizeDegrees(heading + other.heading))

    operator fun plus(other: Vec2d): Pose2d = Pose2d(position + other, heading)

    operator fun minus(other: Pose2d): Pose2d = Pose2d(position - other.position, normalizeDegrees(heading - other.heading))

    operator fun minus(other: Vec2d): Pose2d = Pose2d(position - other, heading)

    operator fun times(other: Vec2d): Pose2d = Pose2d(position * other, heading)

    operator fun times(scalar: Double): Pose2d = Pose2d(position * scalar, heading)

    operator fun div(other: Vec2d): Pose2d = Pose2d(position / other, heading)

    operator fun div(scalar: Double): Pose2d = Pose2d(position / scalar, heading)

    fun distanceTo(pose: Pose2d) = distanceTo(pose.position)

    fun distanceTo(vec2d: Vec2d) = position.distanceTo(vec2d)

    fun sign() = Pose2d(position.sign(), heading.sign)
}

fun Pose2d.toPedro(): Pose {
    return Pose(x, y, radians)
}