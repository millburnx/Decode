package org.firstinspires.ftc.teamcode.common.hardware.gamepad

class Gamepad(private val gamepad: com.qualcomm.robotcore.hardware.Gamepad) {
    var current: GamepadState = GamepadState(gamepad)
    var prev: GamepadState = GamepadState(gamepad)
}