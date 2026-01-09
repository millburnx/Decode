package org.firstinspires.ftc.teamcode.common.hardware

import com.millburnx.util.Pose2d
import com.millburnx.util.toRadians
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

fun motorSetup(motor: DcMotorEx, reverse: Boolean = false, float: Boolean = false) {
    motor.zeroPowerBehavior = if (float) DcMotor.ZeroPowerBehavior.FLOAT else DcMotor.ZeroPowerBehavior.BRAKE
    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
    motor.direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
}

fun Pose2d.toPedro() = Pose(x, y, heading.toRadians())