package org.firstinspires.ftc.teamcode.common.util

import com.millburnx.util.Pose2d
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D

object LimelightUtil {
    fun mtToPedro(pose: Pose3D): Pose2d {
        val inches = pose.position.toUnit(DistanceUnit.INCH)

        val x = 72.0 + inches.y
        val y = 72.0 - inches.x
        val h = 90.0 + pose.orientation.getYaw(AngleUnit.DEGREES)

        return Pose2d(x, y, h)
    }
}