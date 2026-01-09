package org.firstinspires.ftc.teamcode.common.hardware

import com.millburnx.util.Pose2d
import com.millburnx.util.toDegrees
import com.millburnx.util.toRadians
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

fun motorSetup(motor: DcMotorEx, reverse: Boolean = false, float: Boolean = false) {
    motor.zeroPowerBehavior = if (float) DcMotor.ZeroPowerBehavior.FLOAT else DcMotor.ZeroPowerBehavior.BRAKE
    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
    motor.direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
}

fun Pose2d.toPedro() = Pose(x, y, heading.toRadians())

fun Pose2d.Companion.fromPedro(pose: Pose) = Pose2d(pose.x, pose.y, pose.heading.toDegrees())

fun normalizeRadians(radians: Double): Double {
    return atan2(sin(radians), cos(radians))
}

fun normalizeDegrees(angle: Double): Double = Math.toDegrees(normalizeRadians(Math.toRadians(angle)))