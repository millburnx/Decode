package org.firstinspires.ftc.teamcode.common.hardware

import com.bylazar.gamepad.PanelsGamepad
import com.millburnx.cmdxpedro.util.geometry.vector.Vec2d
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode
import org.firstinspires.ftc.teamcode.opmode.OpMode

class GamepadManager(opMode: OpMode) {
    private val _panelsGamepad1: com.bylazar.gamepad.GamepadManager = PanelsGamepad.firstManager
    private val _panelsGamepad2: com.bylazar.gamepad.GamepadManager = PanelsGamepad.secondManager

    private val _gamepad1: com.qualcomm.robotcore.hardware.Gamepad
        get() = _panelsGamepad1.asCombinedFTCGamepad(opMode.gamepad1)
    private val _gamepad2: com.qualcomm.robotcore.hardware.Gamepad
        get() = _panelsGamepad2.asCombinedFTCGamepad(opMode.gamepad2)

    val gamepad1: Gamepad = Gamepad(_gamepad1)
    val gamepad2: Gamepad = Gamepad(_gamepad2)

    fun update() {
        val oldState1 = gamepad1.current
        val oldState2 = gamepad2.current

        val newState1 = GamepadState(_gamepad1)
        val newState2 = GamepadState(_gamepad2)
        gamepad1.current = newState1
        gamepad2.current = newState2

        gamepad1.prev = oldState1
        gamepad2.prev = oldState2
    }
}

class Gamepad(private val gamepad: com.qualcomm.robotcore.hardware.Gamepad) {
    var current: GamepadState = GamepadState(gamepad)
    var prev: GamepadState = GamepadState(gamepad)
}

class GamepadState(private val gamepad: com.qualcomm.robotcore.hardware.Gamepad) {
    val leftJoyStick = JoyStick(
        gamepad.left_stick_x.toDouble(),
        gamepad.left_stick_y.toDouble(),
        gamepad.left_stick_button
    )

    val rightJoyStick = JoyStick(
        gamepad.right_stick_x.toDouble(),
        gamepad.right_stick_y.toDouble(),
        gamepad.right_stick_button
    )

    val dPad = DPad(
        gamepad.dpad_left,
        gamepad.dpad_up,
        gamepad.dpad_right,
        gamepad.dpad_down
    )

    val a: Boolean = gamepad.a
    val b: Boolean = gamepad.b
    val x: Boolean = gamepad.x
    val y: Boolean = gamepad.y

    val guide: Boolean = gamepad.guide
    val start: Boolean = gamepad.start
    val back = gamepad.back

    val leftBumper: Boolean = gamepad.left_bumper
    val rightBumper: Boolean = gamepad.right_bumper

    val leftTrigger: Double = gamepad.left_trigger.toDouble()
    val rightTrigger: Double = gamepad.right_trigger.toDouble()
}

class JoyStick(var x: Double, var y: Double, var down: Boolean) {
    val vector: Vec2d
        get() = Vec2d(x, y)
}

class DPad(var left: Boolean, var up: Boolean, var right: Boolean, var down: Boolean) {
    val vector: Vec2d
        get() {
            val x = (if (right) 1.0 else 0.0) - (if (left) 1.0 else 0.0)
            val y = (if (up) 1.0 else 0.0) - (if (down) 1.0 else 0.0)
            return Vec2d(x, y)
        }
}