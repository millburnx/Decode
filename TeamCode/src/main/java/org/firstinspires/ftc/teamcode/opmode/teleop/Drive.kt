package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants

@Configurable
@TeleOp
class Drive : OpMode() {
    override fun run() {
        scheduler.schedule(Command() {
            val follower = Constants.createFollower(opMode.hardwareMap).apply {
                setStartingPose(Pose(0.0, 0.0))
            }
            while (!isStarted) {
                sync()
            }
            follower.startTeleopDrive(true)
            while (!isStopRequested) {
                follower.update()
                follower.setTeleOpDrive(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble(),
                    true // Robot Centric
                );
            }
        })
    }
}