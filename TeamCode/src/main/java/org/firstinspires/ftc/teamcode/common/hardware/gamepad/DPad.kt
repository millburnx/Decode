package org.firstinspires.ftc.teamcode.common.hardware.gamepad

import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d

class DPad(var left: Boolean, var up: Boolean, var right: Boolean, var down: Boolean) {
    val vector: Vec2d
        get() {
            val x = (if (right) 1.0 else 0.0) - (if (left) 1.0 else 0.0)
            val y = (if (up) 1.0 else 0.0) - (if (down) 1.0 else 0.0)
            return Vec2d(x, y)
        }
}