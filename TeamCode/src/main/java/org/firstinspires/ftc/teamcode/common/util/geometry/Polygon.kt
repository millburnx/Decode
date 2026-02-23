package org.firstinspires.ftc.teamcode.common.util.geometry

import com.millburnx.util.vector.Vec2d

/**
 * All points must be either cw or ccw
 */
data class Polygon(val points: List<Vec2d>) {
    constructor(vararg points: Vec2d) : this(points.toList())

    val lines
        get() = points.indices.map { i ->
            LineSegment(points[i], points[(i + 1) % points.size])
        }

    /**
     * Checks if the point is inside the polygon using the cross product method.
     * Convex polygons only `*C`
     */
    fun contains(point: Vec2d): Boolean {
        val signs = lines.map { (it.p2 - it.p1).cross(point - it.p1) }
        return signs.all { it > 0 } || signs.all { it < 0 }
    }

    fun closestPoint(point: Vec2d): Vec2d {
        return lines.map { it.project(point) }.minBy { it.distance(point) }
    }

    companion object {
        /**
         * Automatically orders the points using graham scan `*C`
         */
        fun autoConvex(points: List<Vec2d>): Polygon {
            if (points.size <= 2) return Polygon(points)

            val pivot = points.minWith(compareBy({ it.y }, { it.x }))

            val sorted = points
                .filter { it != pivot }
                .sortedWith(
                    compareBy(
                        { (it - pivot).angle() },
                        { (it - pivot).magnitude() }
                    ))
                .let { listOf(pivot) + it }

            val hull = ArrayDeque<Vec2d>()
            for (p in sorted) {
                while (hull.size >= 2) {
                    val a = hull[hull.size - 2]
                    val b = hull[hull.size - 1]
                    if ((b - a).cross(p - a) <= 0) hull.removeLast()
                    else break
                }
                hull.addLast(p)
            }

            return Polygon(hull.toList())
        }

        fun autoConvex(vararg points: Vec2d): Polygon = autoConvex(points.toList())
    }
}