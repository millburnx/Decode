package org.firstinspires.ftc.teamcode.opmode

import com.bylazar.telemetry.PanelsTelemetry
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.hardware.gamepad.Gamepad
import org.firstinspires.ftc.teamcode.common.hardware.gamepad.GamepadManager
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualManager
import org.firstinspires.ftc.teamcode.common.subsystem.SubsystemManager

abstract class OpMode : LinearOpMode() {
    val tel = PanelsTelemetry.telemetry

    lateinit var gamepadManager: GamepadManager

    val gp1: Gamepad
        get() = gamepadManager.gamepad1
    val gp2: Gamepad
        get() = gamepadManager.gamepad2

    lateinit var voltageSensor: VoltageSensor

    val loopTimer = ElapsedTime()
    var deltaTime = 0.0
    var hubs: List<LynxModule> = emptyList()
    val scheduler = CommandScheduler().apply {
        onSync = {
            val ms = loopTimer.milliseconds()
            val loopHertz = 1.0 / loopTimer.seconds()
            deltaTime = loopHertz
            loopTimer.reset()

            tel.addData("hz", loopHertz)
            tel.addData("ms", ms)
            tel.update(telemetry)

            // hardware
            hubs.forEach { it.clearBulkCache() }
            ManualManager.update()
            if (::gamepadManager.isInitialized) {
                gamepadManager.update()
            }
        }
    }

    // all code runs here, it is before the wait for start
    // so for code that runs afterward, use a command with a WaitFor blocker
    abstract fun run()

    override fun runOpMode() {
        telemetry.isAutoClear = true

        voltageSensor = hardwareMap.voltageSensor.get("Control Hub")
        gamepadManager = GamepadManager(this)

        SubsystemManager.init()
        run()
        SubsystemManager.registerAll(scheduler)

        waitForStart()

        @Suppress("ControlFlowWithEmptyBody") // Loop is to keep the active mode running
        while (opModeIsActive()) {
        }
        scheduler.runner.cancel() // Clean up faulty commands
//        PanelsCameraStream.stopStream()
    }
}