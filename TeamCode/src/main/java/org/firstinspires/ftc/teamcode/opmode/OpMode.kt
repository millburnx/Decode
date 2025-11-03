package org.firstinspires.ftc.teamcode.opmode

import com.bylazar.gamepad.GamepadManager
import com.bylazar.gamepad.PanelsGamepad
import com.bylazar.telemetry.PanelsTelemetry
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualManager
import org.firstinspires.ftc.teamcode.common.subsystem.SubsystemManager

abstract class OpMode : LinearOpMode() {
    val tel = PanelsTelemetry.telemetry
    val gm1: GamepadManager = PanelsGamepad.firstManager
    val gm2: GamepadManager = PanelsGamepad.secondManager

    val gp1: Gamepad
        get() = gm1.asCombinedFTCGamepad(gamepad1)
    val gp2: Gamepad
        get() = gm2.asCombinedFTCGamepad(gamepad2)

    val loopTimer = ElapsedTime()
    var hubs: List<LynxModule> = emptyList()
    val scheduler = CommandScheduler().apply {
        onSync = {
            val loopHertz = 1.0 / loopTimer.seconds()
            loopTimer.reset()

            tel.addData("hz", loopHertz)
            tel.update(telemetry)

            // hardware
            hubs.forEach { it.clearBulkCache() }
            ManualManager.update()
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
    }
}