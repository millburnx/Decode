package org.firstinspires.ftc.teamcode.opmode.test.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.mirror
import com.millburnx.util.Pose2d
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Hood
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.sorter.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop.Companion.downDuration
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop.Companion.upDuration

@Configurable
@TeleOp
class ShooterTester : OpMode() {
    override fun run() {
        val drive = Drive(this)
        val flyWheel = FlyWheel(this)
        val hood = Hood(this)
        val sorter = Sorter(this)
        val intake = Intake(this)

        drive.pose = Pose2d(112.0, 137.0, -90.0).mirror()

        scheduler.schedule(
            Command("teleop loop")
            {
                OpModeLoop(this@ShooterTester) {
                    flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
                    flyWheel.shootingRPM = targetRPM

                    hood.target = hoodTarget

                    intake.power = intakePower

                    if (!gp1.prev.rightBumper && gp1.current.rightBumper) {
                        scheduler.schedule(Command {
                            sorter.frontPod.isUp = true
                            SleepFor { upDuration}
                            sorter.frontPod.isUp = false
                            SleepFor { downDuration }

                            sorter.sidePod.isUp = true
                            SleepFor { upDuration }
                            sorter.sidePod.isUp = false
                            SleepFor { downDuration }

                            sorter.backPod.isUp = true
                            SleepFor { upDuration }
                            sorter.backPod.isUp = false
                            SleepFor { downDuration }
                        })
                    }

                    val goal = Vec2d(144.0-4.0, 144.0-4.0).mirror()
                    val pose = drive.pose
                    val dist = pose.distanceTo(goal)
                    tel.addData("dist", dist)
                }
            })
    }

    companion object {

        @JvmField
        var targetRPM = 0.0

        @JvmField
        var hoodTarget = 0.0

        @JvmField
        var intakePower = 0.5
    }
}