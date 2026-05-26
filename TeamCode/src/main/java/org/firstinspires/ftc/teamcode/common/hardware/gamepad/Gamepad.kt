package org.firstinspires.ftc.teamcode.common.hardware.gamepad

class Gamepad(private val gamepad: com.qualcomm.robotcore.hardware.Gamepad) {
    var current: GamepadState = GamepadState(gamepad)
    var prev: GamepadState = GamepadState(gamepad)

    val a = GamepadHooks()
    val b = GamepadHooks()
    val x = GamepadHooks()
    val y = GamepadHooks()

    val guide = GamepadHooks()
    val start = GamepadHooks()
    val back = GamepadHooks()

    val leftBumper = GamepadHooks()
    val rightBumper = GamepadHooks()

    val dPad = DPadHooks()

    private val map = listOf(
        ButtonMapping({ prev.a }, { current.a }, a),
        ButtonMapping({ prev.b }, { current.b }, b),
        ButtonMapping({ prev.x }, { current.x }, x),
        ButtonMapping({ prev.y }, { current.y }, y),
        ButtonMapping({ prev.guide }, { current.guide }, guide),
        ButtonMapping({ prev.start }, { current.start }, start),
        ButtonMapping({ prev.back }, { current.back }, back),
        ButtonMapping({ prev.leftBumper }, { current.leftBumper }, leftBumper),
        ButtonMapping({ prev.rightBumper }, { current.rightBumper }, rightBumper),
        ButtonMapping({ prev.dPad.left }, { current.dPad.left }, dPad.left),
        ButtonMapping({ prev.dPad.up }, { current.dPad.up }, dPad.up),
        ButtonMapping({ prev.dPad.right }, { current.dPad.right }, dPad.right),
        ButtonMapping({ prev.dPad.down }, { current.dPad.down }, dPad.down),
    )

    fun triggerHooks() {
        for ((getPrev, getCurrent, hooks) in map) {
            val prev = getPrev()
            val current = getCurrent()
            if (!prev && current) hooks.pressHooks.forEach { it() }
            if (prev && !current) hooks.releaseHooks.forEach { it() }
        }
    }
}

data class ButtonMapping(
    val prevState: () -> Boolean,
    val currentState: () -> Boolean,
    val hooks: GamepadHooks
)

class GamepadHooks {
    val pressHooks = mutableListOf<() -> Unit>()
    val releaseHooks = mutableListOf<() -> Unit>()

    fun onPress(callback: () -> Unit) = pressHooks.add(callback)
    fun onRelease(callback: () -> Unit) = releaseHooks.add(callback)
}

class DPadHooks {
    val left = GamepadHooks()
    val up = GamepadHooks()
    val right = GamepadHooks()
    val down = GamepadHooks()
}