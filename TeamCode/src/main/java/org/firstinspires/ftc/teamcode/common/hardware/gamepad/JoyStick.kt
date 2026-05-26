package org.firstinspires.ftc.teamcode.common.hardware.gamepad

import com.millburnx.util.toDegrees
import com.millburnx.util.vector.Vec2d

class JoyStick(var x: Double, var y: Double, var down: Boolean) {
    val vector: Vec2d
        get() = Vec2d(x, y)

    val magnitude: Double
        get() = vector.magnitude()

    val degrees: Double
        get() = radians.toDegrees()

    val radians: Double
        get() = vector.angle()
}