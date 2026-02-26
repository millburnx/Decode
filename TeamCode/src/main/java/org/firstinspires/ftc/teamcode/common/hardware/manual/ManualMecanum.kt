package org.firstinspires.ftc.teamcode.common.hardware.manual

import com.pedropathing.ftc.FollowerBuilder
import com.pedropathing.ftc.drivetrains.Mecanum
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.common.hardware.cached.CachedMotor


class ManualMecanum(val hardwareMap: HardwareMap, mecanumConstants: MecanumConstants) :
    Mecanum(hardwareMap, mecanumConstants) {
    private val leftFront: ManualMotor = ManualMotor(
        hardwareMap,
        mecanumConstants.leftFrontMotorName,
        constants.leftFrontMotorDirection == DcMotorSimple.Direction.REVERSE,
        true
    )
    private val leftRear: ManualMotor = ManualMotor(
        hardwareMap,
        mecanumConstants.leftRearMotorName,
        constants.leftRearMotorDirection == DcMotorSimple.Direction.REVERSE,
        true
    )

    private val rightFront: ManualMotor = ManualMotor(
        hardwareMap,
        mecanumConstants.rightFrontMotorName,
        constants.rightFrontMotorDirection == DcMotorSimple.Direction.REVERSE,
        true
    )

    private val rightRear: ManualMotor = ManualMotor(
        hardwareMap,
        mecanumConstants.rightRearMotorName,
        constants.rightRearMotorDirection == DcMotorSimple.Direction.REVERSE,
        true
    )

    private val motors: List<CachedMotor> = listOf(leftFront, leftRear, rightFront, rightRear)


    override fun runDrive(drivePowers: DoubleArray) {
        for (i in motors.indices) {
            motors[i].power = -drivePowers[i]
        }
    }
}

fun FollowerBuilder.manualMecanumDrivetrain(hardwareMap: HardwareMap, mecanumConstants: MecanumConstants): FollowerBuilder {
    return setDrivetrain(ManualMecanum(hardwareMap, mecanumConstants))
}