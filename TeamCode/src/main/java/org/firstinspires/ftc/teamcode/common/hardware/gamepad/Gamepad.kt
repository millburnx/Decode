package org.firstinspires.ftc.teamcode.common.hardware.gamepad

class Gamepad(
    private val gamepad: com.qualcomm.robotcore.hardware.Gamepad,
) {
    var current: GamepadState = GamepadState(gamepad)
    var prev: GamepadState = GamepadState(gamepad)

    val a = GamepadHooks { prev.a to current.a }
    val b = GamepadHooks { prev.b to current.b }
    val x = GamepadHooks { prev.x to current.x }
    val y = GamepadHooks { prev.y to current.y }

    val guide = GamepadHooks { prev.guide to current.guide }
    val start = GamepadHooks { prev.start to current.start }
    val back = GamepadHooks { prev.back to current.back }

    val leftBumper = GamepadHooks { prev.leftBumper to current.leftBumper }
    val rightBumper = GamepadHooks { prev.rightBumper to current.rightBumper }

    val dPad = DPadHooks { prev.dPad to current.dPad }

    val leftJoyStick = GamepadHooks { prev.leftJoyStick.down to current.leftJoyStick.down }
    val rightJoyStick = GamepadHooks { prev.rightJoyStick.down to current.rightJoyStick.down }
}

// Pair<Prev, Current>
class GamepadHooks(
    val state: () -> Pair<Boolean, Boolean>,
) {
    fun ifPressed(callback: () -> Unit) {
        val state = state()
        if (!state.first && state.second) callback()
    }

    fun ifReleased(callback: () -> Unit) {
        val state = state()
        if (state.first && !state.second) callback()
    }
}

// Pair<Prev, Current>
class DPadHooks(
    val state: () -> Pair<DPad, DPad>,
) {
    private val prev
        get() = state().first

    private val current
        get() = state().second

    val left = GamepadHooks({ prev.left to current.left })
    val up = GamepadHooks({ prev.left to current.up })
    val right = GamepadHooks({ prev.right to current.right })
    val down = GamepadHooks({ prev.down to current.down })
}
