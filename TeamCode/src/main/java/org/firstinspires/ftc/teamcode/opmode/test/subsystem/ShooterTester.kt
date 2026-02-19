package org.firstinspires.ftc.teamcode.opmode.test.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.util.Pose2d
import com.millburnx.util.vector.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.hardware.fromPedro
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Hood
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Sorter
import org.firstinspires.ftc.teamcode.common.util.OpModeLoop
import org.firstinspires.ftc.teamcode.opmode.OpMode
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop
import org.firstinspires.ftc.teamcode.pedro.Constants

@Configurable
@TeleOp
class ShooterTester : OpMode() {
    override fun run() {
        val pedro = Constants.createManualFusionFollower(hardwareMap, { deltaTime })
        val flyWheel = FlyWheel(this)
        val hood = Hood(this)
        val sorter = Sorter(this)
        val intake = Intake(this)

        scheduler.schedule(
            Command("teleop loop")
            {
                OpModeLoop(this@ShooterTester) {
                    pedro.update()

                    flyWheel.state = FlyWheel.FlyWheelState.SHOOTING
                    flyWheel.shootingRPM = targetRPM

                    hood.target = hoodTarget

                    intake.power = intakePower

                    if (!gp1.prev.rightBumper && gp1.current.rightBumper) {
                        scheduler.schedule(Command {
                            sorter.isFrontUp = true
                            SleepFor { Teleop.upDuration }
                            sorter.isFrontUp = false
                        })
                    }

                    val goal = Vec2d(72.0-6.0, 72.0-6.0)
                    val pose = Pose2d.fromPedro(pedro.pose)
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