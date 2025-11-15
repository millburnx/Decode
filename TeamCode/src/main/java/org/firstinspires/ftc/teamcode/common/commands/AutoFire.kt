package org.firstinspires.ftc.teamcode.common.commands

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdx.commandGroups.Sequential
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import kotlinx.coroutines.isActive
import org.firstinspires.ftc.teamcode.common.commands.AutoFireSettings.firstBallDelay
import org.firstinspires.ftc.teamcode.common.commands.AutoFireSettings.intakeDuration
import org.firstinspires.ftc.teamcode.common.commands.AutoFireSettings.intakePower
import org.firstinspires.ftc.teamcode.common.commands.AutoFireSettings.jamDuration
import org.firstinspires.ftc.teamcode.common.commands.AutoFireSettings.postFinishDuration
import org.firstinspires.ftc.teamcode.common.commands.AutoFireSettings.postLoadDuration
import org.firstinspires.ftc.teamcode.common.commands.AutoFireSettings.unjamDuration
import org.firstinspires.ftc.teamcode.common.commands.AutoFireSettings.unjamPower
import org.firstinspires.ftc.teamcode.common.subsystem.AutoTargeting
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Uppies
import org.firstinspires.ftc.teamcode.opmode.OpMode

fun TeleopAutoFire(opMode: OpMode, autoTargeting: AutoTargeting, intake: Intake, flyWheel: FlyWheel, uppies: Uppies): Command =
    Command("Auto Fire Triggerer") {
        with(opMode) {
            val autoFire = AutoFire(this, autoTargeting, intake, flyWheel, uppies, isTeleop = true)
            WaitFor { isStarted || isStopRequested }
            while (!isStopRequested) {
                if (gp1.current.a && !gp1.prev.a) {
                    if (autoFire.currentScope?.isActive ?: false) {
//                        // already scheduled, cancel
//                        autoFire.cancel()
//                        scheduler.schedule(Command {
//                            sync()
//                            unlock(intake, flyWheel, uppies)
//                            intake.power = 0.0
//                            flyWheel.state = FlyWheel.State.IDLE
//                        })
                    } else {
                        scheduler.schedule(autoFire)
                    }
                }
                sync()
            }
        }
    }

fun AutoFire(opMode: OpMode, autoTargeting: AutoTargeting?, intake: Intake, flyWheel: FlyWheel, uppies: Uppies, isTeleop: Boolean = false): Sequential {
    with(opMode) {
        suspend fun Command.shootBall() {
            WaitFor { (flyWheel.atVelocity && uppies.atTarget) || isStopRequested }
            uppies.nextState()
        }

        suspend fun Command.attemptUnjam() {
            uppies.prevState()
            intake.power = -unjamPower
            SleepFor({ isStopRequested }) { unjamDuration }
        }

        suspend fun Command.attemptLoad() {
            intake.power = intakePower
            SleepFor({ isStopRequested }) { intakeDuration }
            uppies.nextState()
            // check for a jam
        }

        suspend fun Command.isJammed(): Boolean {
            SleepFor({ isStopRequested || uppies.atTarget }) { jamDuration }
            return !uppies.atTarget
        }

        suspend fun Command.loadBall(jamProtection: Boolean = AutoFireSettings.jamProtection) {
            if (!jamProtection) {
                intake.power = 0.5
                SleepFor({ isStopRequested }) { intakeDuration }
                uppies.nextState()
            } else {
                attemptLoad()
                while (isJammed()) {
                    attemptUnjam()
                    attemptLoad()
                }
            }
            SleepFor { postLoadDuration }
        }

        return Sequential("Auto Fire") {
            Command("Prepare Subsystems") {
                if (!isStarted) parentGroup?.cancel()
                if (isTeleop) lock(intake, flyWheel, uppies)
                intake.power = 0.0
                flyWheel.state = FlyWheel.State.SHOOTING
            }
            Command("Verify First Ball") {
                if (uppies.state != Uppies.State.LOADED) {
                    loadBall()
                }
            }
            Command("First Ball Wait") {
                SleepFor { firstBallDelay }
            }
            Command("Shoot First Ball") { shootBall() }
            Command("Load Second Ball") { loadBall() }
            Command("Shoot Second Ball") { shootBall() }
            Command("Load Third Ball") { loadBall() }
            Command("Shoot Third Ball") { shootBall() }
            Command("Reset Subsystems") {
                SleepFor { postFinishDuration }
                intake.power = 0.0
                flyWheel.state = FlyWheel.State.IDLE
                if (isTeleop) unlock(intake, flyWheel, uppies)
                autoTargeting?.enabled = false
            }
        }
    }
}

@Configurable
object AutoFireSettings {
    @JvmField
    var intakeDuration = 500L

    @JvmField
    var intakePower = 0.8

    @JvmField
    var jamDuration = 500L

    @JvmField
    var unjamDuration = 250L

    @JvmField
    var unjamPower = -0.1

    @JvmField
    var firstBallDelay = 1000L

    @JvmField
    var postLoadDuration = 500L

    @JvmField
    var jamProtection = true

    @JvmField
    var postFinishDuration = 1000L
}

//fun AutoFire(opMode: OpMode, intake: Intake, flyWheel: FlyWheel, uppies: Uppies) = Command("AutoFire") {
//    with(opMode) {
//        require(isStarted)
//        lock(intake, flyWheel, uppies)
//        intake.power = 0.0
//        flyWheel.state = FlyWheel.State.SHOOTING
//        WaitFor { flyWheel.atVelocity }
//        uppies.nextState()
//        unlock(intake, flyWheel, uppies)
//    }
//}

private fun lock(intake: Intake, flyWheel: FlyWheel, uppies: Uppies) {
    intake.isTeleop = false
    flyWheel.isTeleop = false
    uppies.isTeleop = false
}

private fun unlock(intake: Intake, flyWheel: FlyWheel, uppies: Uppies) {
    intake.isTeleop = true
    flyWheel.isTeleop = true
    uppies.isTeleop = true
}