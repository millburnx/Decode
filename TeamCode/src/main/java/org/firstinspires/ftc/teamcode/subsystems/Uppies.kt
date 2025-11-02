package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDController
import com.bylazar.configurables.annotations.Configurable
import com.millburnx.cmdx.Command
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.util.ManualAxon
import org.firstinspires.ftc.teamcode.util.SleepFor
import org.firstinspires.ftc.teamcode.util.WaitFor

class AxonCR(
    hardwareMap: HardwareMap,
    name: String,
    encoderName: String,
    reverse: Boolean = false,
    val encoderReverse: Boolean = false
) {
    val servo: CRServo = hardwareMap.crservo[name].apply {
        direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    }
    var power
        get() = servo.power
        set(power) {
            servo.power = power
        }

    val encoder = hardwareMap.analogInput[encoderName]
    var rawPosition: Double = 0.0 //kedaar wuz here
    var rotations: Int = 0 //kedaar wuz also here
    val position
        get() = rotations + rawPosition

    fun updatePosition(): Double {
        val oldPosition = rawPosition
        val raw = encoder.voltage / 3.3
        if (encoderReverse) rawPosition = raw else rawPosition = 1 - raw

        val angleDifference: Double = rawPosition - oldPosition
        val threshold = 0.5

        // Handle wraparound at 0|1 boundary
        if (angleDifference < -threshold) {
            rotations++ // 1 to 0
        } else if (angleDifference > threshold) {
            rotations-- // 0 to 1
        }

        return position
    }
}

@Configurable
class Uppies(opMode: LinearOpMode, intake: Intake, flyWheel: FlyWheel) : Subsystem("Uppies") {
    val left = ManualAxon(opMode.hardwareMap, "s0", "a0", reverse = false, encoderReverse = false)
    val right = ManualAxon(opMode.hardwareMap, "s1", "a1", reverse = true, encoderReverse = true)

    var leftState: Positions = Positions.OPEN;
    var rightState: Positions = Positions.OPEN;

    var state: States = States.LEFT_SHOT

    val states = listOf<States>(
        States.LEFT_LOADED,
        States.LEFT_SHOT,
//        States.RIGHT_LOADED,
//        States.RIGHT_SHOT,
    )

    val pidLeft = PIDController(kP, kI, kD)
    val pidRight = PIDController(kP, kI, kD)

    var leftRotations = 0
    var rightRotations = 0

    fun next() {
        state = states[(states.indexOf(state) + 1) % states.size]
        when (state) {
            States.LEFT_LOADED -> leftState = Positions.LOADED
            States.LEFT_SHOT -> {
                leftRotations++
                leftState = Positions.OPEN
            }

//            States.RIGHT_LOADED -> rightState = Positions.LOADED
//            States.RIGHT_SHOT -> {
//                rightRotations++
//                rightState = Positions.OPEN
//            }
        }
    }

    fun prev() {
        if (state == States.LEFT_LOADED) {
            state = States.LEFT_SHOT
            leftState = Positions.OPEN
            rightState = Positions.OPEN
        }
//        if (state == States.RIGHT_LOADED) {
//            state = States.LEFT_SHOT
//            leftState = Positions.OPEN
//            rightState = Positions.OPEN
//        }
    }

    val loadBall: suspend Command.() -> Unit = {
        intake.power = -1.0 // run intake
        SleepFor(intakeDuration)
        next() // load 2
        SleepFor(250)
        intake.power = -0.5
        WaitFor { atPosition() }
        intake.power = 0.0 // stop intake
    }

    val autoFireCommand = Command("auto-fire", {
        println("uppies log | cancelling")
        intake.power = 0.0
        intake.locked = false
        flyWheel.running = false
    }) {
        intake.locked = true
        flyWheel.running = true
        intake.power = 0.0
        WaitFor { atPosition() }
        SleepFor(flyWheelDuration)
        next() // fire first ball
        WaitFor { atPosition() }
        println("uppies log | firing first ball | $state")
        loadBall()
        println("uppies log | load second ball | $state")
        next() // fire 2
        WaitFor { atPosition() }
        println("uppies log | firing second ball | $state")
        loadBall()
        println("uppies log | load third ball | $state")
        next() // fire 3
        WaitFor { atPosition() }
        println("uppies log | firing third ball | $state")
        intake.locked = false
        flyWheel.running = false
    }

    val autoFireCommandTeleop = Command("auto-fire-teleop", {
        println("uppies log | cancelling")
        intake.power = 0.0
        intake.locked = false
        flyWheel.running = false
    }) {
        intake.locked = true
        flyWheel.running = true
        intake.power = 0.0
        WaitFor { atPosition() }
        SleepFor(flyWheelDuration)
        next() // fire first ball
        WaitFor { atPosition() }
        println("uppies log | firing first ball | $state")
        loadBall()
        println("uppies log | load second ball | $state")
        next() // fire 2
        WaitFor { atPosition() }
        println("uppies log | firing second ball | $state")
//        loadBall()
//        println("uppies log | load third ball | $state")
//        next() // fire 3
//        WaitFor { atPosition() }
//        println("uppies log | firing third ball | $state")
        intake.locked = false
        flyWheel.running = false
    }

    val loadBallNew: suspend Command.() -> Unit = {
        intake.power = -1.0 // run intake
        SleepFor(intakeDuration)
        next() // load 2
        SleepFor(250)
        intake.power = -0.5
        uppiesValidator()
        intake.power = 0.0 // stop intake
    }

    val uppiesValidator: suspend Command.() -> Unit = {
        val elapsedTime = ElapsedTime()
        while (!atPosition() && elapsedTime.milliseconds() < 3000) {
            sync()
        }
        if (!atPosition()) {
            // shit is stuck tspmo
            prev()
            intake.power = 0.5
            SleepFor(500) // reverse intake
            // ---
            intake.power = -1.0 // run intake
            SleepFor(intakeDuration)
            next() // load 2
            SleepFor(250)
            intake.power = -0.5
            SleepFor(500)
            intake.power = 0.0
        }
    }

    val autoFireCommandNew = Command("auto-fire-new", {
        println("uppies log | cancelling")
        intake.power = 0.0
        intake.locked = false
        flyWheel.running = false
    }) {
        intake.locked = true
        flyWheel.running = true
        intake.power = 0.0
        uppiesValidator()
        SleepFor(flyWheelDuration)
        next() // fire first ball
        uppiesValidator()
        println("uppies log | firing first ball | $state")
        loadBallNew()
        println("uppies log | load second ball | $state")
        next() // fire 2
        uppiesValidator()
        println("uppies log | firing second ball | $state")
        loadBallNew()
        println("uppies log | load third ball | $state")
        next() // fire 3
        uppiesValidator()
        println("uppies log | firing third ball | $state")
        intake.locked = false
        flyWheel.running = false
    }

    fun atPosition(): Boolean {
        val leftError = left.position - (leftRotations + leftState.getPosition(true))
        val rightError = right.position - (rightRotations + rightState.getPosition(false))

        return leftError > threshold && rightError > threshold
    }

    override val run: suspend Command.() -> Unit = {
        with(opMode) {
            var prevRevButton = gamepad1.left_bumper
            var prevAdvButton = gamepad1.right_bumper
            while (!isStopRequested) {
                val newRevButton = gamepad1.left_bumper
                val newAdvButton = gamepad1.right_bumper
                if (!prevRevButton && newRevButton) {
                    prev()
                }
                if (!prevAdvButton && newAdvButton) {
                    next()
                }
                prevRevButton = newRevButton
                prevAdvButton = newAdvButton

                val leftTarget = leftRotations + leftState.getPosition(true)
                val rightTarget = rightRotations + rightState.getPosition(false)

                pidLeft.p = kP
                pidLeft.i = kI
                pidLeft.d = kD

                pidRight.p = kP
                pidRight.i = kI
                pidRight.d = kD

                left.power = pidLeft.calculate(left.position, leftTarget)
                right.power = pidRight.calculate(right.position, rightTarget)

                telemetry.addData("state", state)
                telemetry.addData("left state", leftState)
                telemetry.addData("right state", rightState)
                telemetry.addData("left target", leftTarget)
                telemetry.addData("right target", rightTarget)
                telemetry.addData("left raw pos", left.rawPosition)
                telemetry.addData("right raw pos", right.rawPosition)
                telemetry.addData("left pos", left.position)
                telemetry.addData("right pos", right.position)
                sync()
            }
        }
    }

    override val command = Command(this.name, cleanup, run)

    enum class States {
        LEFT_LOADED,
        LEFT_SHOT,
//        RIGHT_LOADED,
//        RIGHT_SHOT
    }

    enum class Positions {
        OPEN, LOADED;

        fun getPosition(left: Boolean): Double = when (this) {
            OPEN -> if (left) openLeft else openRight
            LOADED -> if (left) loadedLeft else loadedRight
        }
    }

    companion object {
        @JvmField
        var kP = 2.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.05

        @JvmField
        var openLeft = 0.14

        @JvmField
        var openRight = 0.125

        @JvmField
        var loadedLeft = 0.43

        @JvmField
        var loadedRight = 0.4

        @JvmField
        var threshold = -0.05

        @JvmField
        var intakeDuration = 1500L

        @JvmField
        var flyWheelDuration = 500L
    }
}