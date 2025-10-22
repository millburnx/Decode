package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.millburnx.cmdx.runtimeGroups.CommandScheduler
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp


@TeleOp(name = "AnalogTest")
class AnalogTest : LinearOpMode() {
    val scheduler = CommandScheduler()
    val tel = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        val a0 = hardwareMap.analogInput["a0"]
        val a1 = hardwareMap.analogInput["a1"]
        val s0 = hardwareMap.crservo["s0"]
        val s1 = hardwareMap.crservo["s1"]

//        telemetry.isAutoClear = true

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            tel.addData("a0", a0.voltage)
            tel.addData("a1", a1.voltage)
            s0.power = 0.1
            s1.power = 0.1
            tel.update()
        }
    }
}
