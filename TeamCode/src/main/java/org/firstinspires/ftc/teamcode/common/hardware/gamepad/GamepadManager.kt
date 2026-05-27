package org.firstinspires.ftc.teamcode.common.hardware.gamepad

import com.bylazar.gamepad.GamepadManager
import com.bylazar.gamepad.PanelsGamepad
import org.firstinspires.ftc.teamcode.opmode.OpMode

class GamepadManager(private val opMode: OpMode) {
    private val _panelsGamepad1: GamepadManager = PanelsGamepad.firstManager
    private val _panelsGamepad2: GamepadManager = PanelsGamepad.secondManager

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