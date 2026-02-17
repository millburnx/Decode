package org.firstinspires.ftc.teamcode.opmode.test.hardware

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode

@TeleOp
@Configurable
class MotorTester : OpMode() {
    override fun run() {
        val m1 = hardwareMap[motorName1] as DcMotorEx
        val m2 = hardwareMap[motorName2] as DcMotorEx

        m1.direction = if (motorReversed1) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
        m2.direction = if (motorReversed2) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD

        m1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        m2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        m1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
        m2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS

        scheduler.schedule(Command {
            OpModeLoop(this@MotorTester) {
                m1.power = power1
                m2.power = power2

                tel.addData("v1", m1.velocity)
                tel.addData("v2", m2.velocity)
                tel.addData("p1", m1.currentPosition)
                tel.addData("p2", m2.currentPosition)
            }
        })
    }

    companion object {
        @JvmField
        var motorName1 = "m0"

        @JvmField
        var motorName2 = "m1"

        @JvmField
        var motorReversed1 = true

        @JvmField
        var motorReversed2 = false

        @JvmField
        var power1 = 0.0

        @JvmField
        var power2 = 0.0
    }
}