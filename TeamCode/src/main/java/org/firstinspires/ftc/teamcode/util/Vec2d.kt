package org.firstinspires.ftc.teamcode.util

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * Represents a 2D vector/point
 */
data class Vec2d(
    val x: Double = 0.0,
    val y: Double = x,
) {
    constructor(x: Number, y: Number) : this(x.toDouble(), y.toDouble())
    constructor(arr: Array<Number>) : this(arr[0].toDouble(), arr[1].toDouble())
    constructor(arr: Array<Double>) : this(arr[0], arr[1])

    constructor(v: Double) : this(v, v)
    constructor(v: Float) : this(v.toDouble())
    constructor(v: Int) : this(v.toDouble())

    operator fun plus(other: Vec2d) = Vec2d(x + other.x, y + other.y)

    operator fun plus(other: Double) = Vec2d(x + other, y + other)

    operator fun plus(other: Float) = this + other.toDouble()

    operator fun plus(other: Int) = this + other.toDouble()

    operator fun minus(other: Vec2d) = Vec2d(x - other.x, y - other.y)

    operator fun minus(other: Double) = Vec2d(x - other, y - other)

    operator fun minus(other: Float) = this - other.toDouble()

    operator fun minus(other: Int) = this - other.toDouble()

    operator fun times(other: Vec2d) = Vec2d(x * other.x, y * other.y)

    operator fun times(other: Double) = Vec2d(x * other, y * other)

    operator fun times(other: Float) = this * other.toDouble()

    operator fun times(other: Int) = this * other.toDouble()

    operator fun div(other: Vec2d) = Vec2d(x / other.x, y / other.y)

    operator fun div(other: Double) = Vec2d(x / other, y / other)

    operator fun div(other: Float) = this / other.toDouble()

    operator fun div(other: Int) = this / other.toDouble()

    operator fun unaryMinus() = Vec2d(-x, -y)

    /**
     * Returns the Euclidean distance to another point
     */
    fun distanceTo(other: Vec2d): Double {
        val xDiff = x - other.x
        val yDiff = y - other.y
        return kotlin.math.sqrt(xDiff * xDiff + yDiff * yDiff)
    }

    fun magnituide(): Double = kotlin.math.sqrt(x * x + y * y)

    fun normalize(): Double = kotlin.math.sqrt(this.dot(this))

    fun dot(other: Vec2d): Double = x * other.x + y * other.y

    /**
     * Returns the angle to another point
     */
    fun angleTo(other: Vec2d): Double = atan2(other.y - y, other.x - x)

    override fun toString(): String = "Point($x, $y)"

    /**
     * Rotates the vector by an angle
     */
    fun rotate(angle: Double): Vec2d {
        val cos = cos(angle)
        val sin = sin(angle)
        return Vec2d(x * cos - y * sin, x * sin + y * cos)
    }

    /**
     * Linearly interpolates between two vectors/points
     */
    fun lerp(
        other: Vec2d,
        t: Double,
    ) = this + (other - this) * t

    fun lerp(
        other: Vec2d,
        t: Float,
    ) = this.lerp(other, t.toDouble())

    /**
     * Returns a copy of the vector with the absolute value of each component
     */
    fun abs() = Vec2d(kotlin.math.abs(x), kotlin.math.abs(y))

    fun sqrt() = Vec2d(kotlin.math.sqrt(x), kotlin.math.sqrt(y))

    fun sign() = Vec2d(kotlin.math.sign(x), kotlin.math.sign(y))

    fun coerceIn(
        min: Vec2d,
        max: Vec2d,
    ) = Vec2d(x.coerceIn(min.x, max.x), y.coerceIn(min.y, max.y))

    fun coerceIn(
        min: Double,
        max: Double,
    ) = coerceIn(Vec2d(min), Vec2d(max))

    fun flip() = Vec2d(y, x)

    companion object {
        fun fromAngle(radians: Double) = Vec2d(cos(radians), sin(radians))
    }
}