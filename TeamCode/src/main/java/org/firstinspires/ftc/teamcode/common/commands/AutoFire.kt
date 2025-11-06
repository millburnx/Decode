package org.firstinspires.ftc.teamcode.common.commands

import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.millburnx.cmdxpedro.util.SleepFor
import com.millburnx.cmdxpedro.util.WaitFor
import org.firstinspires.ftc.teamcode.common.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.common.subsystem.Intake
import org.firstinspires.ftc.teamcode.common.subsystem.Uppies
import org.firstinspires.ftc.teamcode.opmode.OpMode

fun AutoFire(opMode: OpMode, intake: Intake, flyWheel: FlyWheel, uppies: Uppies) = Command("AutoFire") {
    with(opMode) {
        require(isStarted)
        lock(intake, flyWheel, uppies)
        intake.power = 0.0
        flyWheel.state = FlyWheel.State.SHOOTING
        WaitFor { flyWheel.atVelocity == true }
        unlock(intake, flyWheel, uppies)
    }
}

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