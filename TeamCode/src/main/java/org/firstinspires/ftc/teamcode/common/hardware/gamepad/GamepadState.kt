package org.firstinspires.ftc.teamcode.common.hardware.gamepad

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