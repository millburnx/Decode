package org.firstinspires.ftc.teamcode.opmode

import com.bylazar.camerastream.PanelsCameraStream
import com.bylazar.telemetry.PanelsTelemetry
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.hardware.Gamepad
import org.firstinspires.ftc.teamcode.common.hardware.GamepadManager
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualManager
import org.firstinspires.ftc.teamcode.common.subsystem.SubsystemManager

abstract class OpMode : LinearOpMode() {
    val tel = PanelsTelemetry.telemetry

    val gamepadManager: GamepadManager = GamepadManager(this)

    val gp1: Gamepad = gamepadManager.gamepad1
    val gp2: Gamepad = gamepadManager.gamepad2

    val loopTimer = ElapsedTime()
    var hubs: List<LynxModule> = emptyList()
    val scheduler = CommandScheduler().apply {
        onSync = {
            val ms = loopTimer.milliseconds()
            val loopHertz = 1.0 / loopTimer.seconds()
            loopTimer.reset()

            tel.addData("hz", loopHertz)
            tel.addData("ms", ms)
            tel.update(telemetry)

            // hardware
            hubs.forEach { it.clearBulkCache() }
            ManualManager.update()
            gamepadManager.update()
        }
    }

    // all code runs here, it is before the wait for start
    // so for code that runs afterward, use a command with a WaitFor blocker
    abstract fun run()

    override fun runOpMode() {
        telemetry.isAutoClear = true

        SubsystemManager.init()
        run()
        SubsystemManager.registerAll(scheduler)

        waitForStart()

        @Suppress("ControlFlowWithEmptyBody") // Loop is to keep the active mode running
        while (opModeIsActive()) {
        }
        scheduler.runner.cancel() // Clean up faulty commands
        PanelsCameraStream.stopStream()
    }
}