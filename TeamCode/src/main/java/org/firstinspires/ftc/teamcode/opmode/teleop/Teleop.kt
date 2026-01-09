package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystem.Drive
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Kickers
import org.firstinspires.ftc.teamcode.opmode.OpMode

@Configurable
@TeleOp

class Teleop : OpMode() {
    override fun run() {
        val drive = Drive(this, isTeleop = true)
        val intake = Intake(this, isTeleop = true)
        val kickers = Kickers(this, isTeleop = true)
//        val turret = Turret(this, isTeleop = true)
        val flyWheel = FlyWheel(this, isTeleop = true)
//        val hood = Hood(this, isTeleop = true)

        val rapidFire = Sequential("rapid fire") {
            Command("start flywheel") {
                flyWheel.targetRpm = FlyWheel.targetRpm
                flyWheel.enabled = true
//                SleepFor { FlyWheel.spinUpDuration }
                WaitFor { flyWheel.atVelocity }
            }
            Command("kick first") {
                kickers.isUp3 = true
                SleepFor { Kickers.upDuration }
                kickers.isUp3 = false
                SleepFor { Kickers.downDuration }
            }
            Command("kick second") {
                kickers.isUp1 = true
                SleepFor { Kickers.upDuration }
                kickers.isUp1 = false
                SleepFor { Kickers.downDuration }
            }
            Command("kick third") {
                kickers.isUp2 = true
                SleepFor { Kickers.upDuration }
                kickers.isUp2 = false
                SleepFor { Kickers.downDuration }
            }
            Command("stop flywheel") {
                flyWheel.enabled = false
            }
        }

        scheduler.schedule(Command("rapid fire scheduler") {
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (gp1.current.rightBumper && !gp1.prev.rightBumper) {
                    scheduler.schedule(rapidFire)
                }
                if (gp1.current.leftBumper && !gp1.prev.leftBumper) {
                    flyWheel.enabled = !flyWheel.enabled
                }
                sync()
            }
        })
    }
}