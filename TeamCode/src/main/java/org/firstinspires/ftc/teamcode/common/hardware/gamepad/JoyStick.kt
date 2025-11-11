package org.firstinspires.ftc.teamcode.common.hardware.gamepad

import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d

class JoyStick(var x: Double, var y: Double, var down: Boolean) {
    val vector: Vec2d
        get() = Vec2d(x, y)
}