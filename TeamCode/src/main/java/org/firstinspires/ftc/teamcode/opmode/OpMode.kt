package org.firstinspires.ftc.teamcode.opmode

import com.bylazar.field.PanelsField
import com.bylazar.telemetry.PanelsTelemetry
import com.millburnx.cmdx.Settings
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.hardware.gamepad.Gamepad
import org.firstinspires.ftc.teamcode.common.hardware.gamepad.GamepadManager
import org.firstinspires.ftc.teamcode.common.hardware.manual.ManualManager
import org.firstinspires.ftc.teamcode.common.subsystem.SubsystemManager
import org.firstinspires.ftc.teamcode.common.util.TimeAverage
import org.firstinspires.ftc.teamcode.pedro.Drawing
import kotlin.time.measureTime

abstract class OpMode : LinearOpMode() {
    val tel = PanelsTelemetry.telemetry

    lateinit var gamepadManager: GamepadManager

    val gp1: Gamepad
        get() = gamepadManager.gamepad1
    val gp2: Gamepad
        get() = gamepadManager.gamepad2

    lateinit var voltageSensor: VoltageSensor

    val loopTimer = ElapsedTime()

    val matchTimer = ElapsedTime()
    var deltaTime = 0.0

    val averageHz = TimeAverage { 1000.0 }
    var hubs: List<LynxModule> = emptyList()
    val scheduler = CommandScheduler().apply {
        Settings.verbose = false;
        onSync = {
            val ms = loopTimer.milliseconds()
            val loopHertz = 1.0 / loopTimer.seconds()
            deltaTime = loopHertz
            loopTimer.reset()

            val syncTimer = ElapsedTime()

//            tel.addData("hz", loopHertz)
//            tel.addData("ms", ms)

            averageHz.update(loopHertz)
            tel.addData("smoothed hz", averageHz.average)

            val telUpdateTime = measureTime {
                tel.update(telemetry)
            }
//            tel.addData("tel ms", telUpdateTime.inWholeMilliseconds)
//            Drawing.sendPacket()

            PanelsField.field.update()

            // hardware
            val bulkReadTime = measureTime {
                hubs.forEach { it.clearBulkCache() }
            }
            val manualUpdateTime = measureTime {
                ManualManager.update()
            }
//            tel.addData("br ms", bulkReadTime.inWholeMilliseconds)
//            tel.addData("mu ms", manualUpdateTime.inWholeMilliseconds)

            if (::gamepadManager.isInitialized) {
                gamepadManager.update()
            }

//            tel.addData("onsync ms", syncTimer.milliseconds())
            tel.addData("match timer", matchTimer.seconds())
            syncTimer.reset()
        }
    }

    // all code runs here, it is before the wait for start
    // so for code that runs afterward, use a command with a WaitFor blocker
    abstract fun run()

    override fun runOpMode() {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        telemetry.isAutoClear = true

        voltageSensor = hardwareMap.voltageSensor.get("Control Hub")
        gamepadManager = GamepadManager(this)

        SubsystemManager.init()
        Drawing.init()
        run()
        SubsystemManager.registerAll(scheduler)

        waitForStart()
        matchTimer.reset()

        @Suppress("ControlFlowWithEmptyBody") // Loop is to keep the active mode running
        while (opModeIsActive()) {
        }
        scheduler.runner.cancel() // Clean up faulty commands
//        PanelsCameraStream.stopStream()
    }
}