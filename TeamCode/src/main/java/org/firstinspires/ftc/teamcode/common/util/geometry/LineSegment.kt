package org.firstinspires.ftc.teamcode.common.util.geometry

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.millburnx.util.vector.Vec2d

data class LineSegment(val p1: Vec2d, val p2: Vec2d) {
    fun project(point: Vec2d): Vec2d {
        val ab = p2 - p1

        val divisor = ab.dot(ab)
        if (divisor == 0.0) return p1

        val t = ((point - p1).dot(ab)) / divisor
        return p1.lerp(p2, t.clamp(0.0, 1.0))
    }
}