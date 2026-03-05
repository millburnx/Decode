package org.firstinspires.ftc.teamcode.pedro

import android.annotation.SuppressLint
import com.bylazar.configurables.PanelsConfigurables
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.configurables.annotations.IgnoreConfigurable
import com.bylazar.field.FieldManager
import com.bylazar.field.PanelsField
import com.bylazar.field.Style
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.pedropathing.math.Vector
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.telemetry.SelectScope
import com.pedropathing.telemetry.SelectableOpMode
import com.pedropathing.util.PoseHistory
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.util.ElapsedTime
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Configurable
@TeleOp(name = "Tuning", group = "Pedro Pathing")
class Tuning : SelectableOpMode(
    "Select a Tuning OpMode",
    Consumer { s: SelectScope<Supplier<OpMode?>?>? ->
        s!!.folder(
            "Localization"
        ) { l: SelectScope<Supplier<OpMode?>?>? ->
            l!!.add("Localization Test", Supplier { LocalizationTest() })
            l.add("Forward Tuner", Supplier { ForwardTuner() })
            l.add("Lateral Tuner", Supplier { LateralTuner() })
            l.add("Turn Tuner", Supplier { TurnTuner() })
        }
        s.folder(
            "Automatic"
        ) { a: SelectScope<Supplier<OpMode?>?>? ->
            a!!.add("Forward Velocity Tuner", Supplier { ForwardVelocityTuner() })
            a.add("Lateral Velocity Tuner", Supplier { LateralVelocityTuner() })
            a.add("Forward Zero Power Acceleration Tuner", Supplier { ForwardZeroPowerAccelerationTuner() })
            a.add("Lateral Zero Power Acceleration Tuner", Supplier { LateralZeroPowerAccelerationTuner() })
            a.add("Predictive Braking Tuner", Supplier { PredictiveBrakingTuner() })
        }
        s.folder(
            "Manual"
        ) { p: SelectScope<Supplier<OpMode?>?>? ->
            p!!.add("Translational Tuner", Supplier { TranslationalTuner() })
            p.add("Heading Tuner", Supplier { HeadingTuner() })
            p.add("Drive Tuner", Supplier { DriveTuner() })
            p.add("Centripetal Tuner", Supplier { CentripetalTuner() })
        }
        s.folder(
            "Tests"
        ) { p: SelectScope<Supplier<OpMode?>?>? ->
            p!!.add("Line", Supplier { Line() })
            p.add("Triangle", Supplier { Triangle() })
            p.add("Circle", Supplier { Circle() })
            p.add("Line90DegreeTurn", Supplier { Line90DegreeTurn() })
        }
        s.folder(
            "Swerve"
        ) { p: SelectScope<Supplier<OpMode?>?>? ->
            p!!.add("Analog Min / Max Tuner", Supplier { AnalogMinMaxTuner() })
            p.add("Swerve Offsets Test", Supplier { SwerveOffsetsTest() })
            p.add("Swerve Turn Test", Supplier { SwerveTurnTest() })
        }
    }) {
    public override fun onSelect() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap)
            PanelsConfigurables.refreshClass(this)
        } else {
            follower = Constants.createFollower(hardwareMap)
        }

        follower!!.setStartingPose(Pose())

        poseHistory = follower!!.poseHistory

        telemetryM = PanelsTelemetry.telemetry

        Drawing.init()
    }

    public override fun onLog(lines: MutableList<String?>?) {}

    companion object {
        var follower: Follower? = null

        @IgnoreConfigurable
        var poseHistory: PoseHistory? = null

        @IgnoreConfigurable
        var telemetryM: TelemetryManager? = null

        @IgnoreConfigurable
        var changes: ArrayList<String?> = ArrayList<String?>()

        fun drawCurrent() {
            try {
                Drawing.drawRobot(follower!!.pose)
                Drawing.sendPacket()
            } catch (e: Exception) {
                throw RuntimeException("Drawing failed $e")
            }
        }

        fun drawCurrentAndHistory() {
            Drawing.drawPoseHistory(poseHistory!!)
            drawCurrent()
        }

        /** This creates a full stop of the robot by setting the drive motors to run at 0 power.  */
        fun stopRobot() {
            follower!!.startTeleopDrive(true)
            follower!!.setTeleOpDrive(0.0, 0.0, 0.0, true)
        }
    }
}

/**
 * This is the LocalizationTest OpMode. This is basically just a simple drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot.
 * You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Kabir Goyal
 * @version 1.0, 5/6/2024
 */
internal class LocalizationTest : OpMode() {
    var debugStringEnabled: Boolean = false

    override fun init() {}

    /** This initializes the PoseUpdater, the drive motors, and the Panels telemetry.  */
    override fun init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled
        }


        Tuning.telemetryM!!.debug(
            "This will print your robot's position to telemetry while "
                    + "allowing robot control through a basic drive on gamepad 1."
        )
        Tuning.telemetryM!!.debug(
            "Drivetrain debug string " + ((if (debugStringEnabled) "enabled" else "disabled")) +
                    " (press gamepad a to toggle)"
        )
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun start() {
        Tuning.follower!!.startTeleopDrive()
        Tuning.follower!!.update()
    }

    /**
     * This updates the robot's pose estimate, the simple drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    override fun loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled
        }

        Tuning.follower!!.setTeleOpDrive(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble(),
            true
        )
        Tuning.follower!!.update()

        Tuning.telemetryM!!.debug("x:" + Tuning.follower!!.pose.x)
        Tuning.telemetryM!!.debug("y:" + Tuning.follower!!.pose.y)
        Tuning.telemetryM!!.debug("heading:" + Tuning.follower!!.pose.heading)
        Tuning.telemetryM!!.debug("total heading:" + Tuning.follower!!.totalHeading)
        if (debugStringEnabled) {
            Tuning.telemetryM!!.debug(
                "Drivetrain Debug String:\n" +
                        Tuning.follower!!.getDrivetrain().debugString()
            )
        }
        Tuning.telemetryM!!.update(telemetry)

        Tuning.drawCurrentAndHistory()
    }
}

/**
 * This is the ForwardTuner OpMode. This tracks the forward movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the forward ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
internal class ForwardTuner : OpMode() {
    override fun init() {
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("Pull your robot forward $DISTANCE inches. Your forward ticks to inches will be shown on the telemetry.")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.drawCurrent()
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        Tuning.follower!!.update()

        Tuning.telemetryM!!.debug("Distance Moved: " + Tuning.follower!!.pose.x)
        Tuning.telemetryM!!.debug("The multiplier will display what your forward ticks to inches should be to scale your current distance to $DISTANCE inches.")
        Tuning.telemetryM!!.debug(
            "Multiplier: " + (DISTANCE / (Tuning.follower!!.pose.x / Tuning.follower!!.getPoseTracker()
                .localizer
                .forwardMultiplier))
        )
        Tuning.telemetryM!!.update(telemetry)

        Tuning.drawCurrentAndHistory()
    }

    companion object {
        var DISTANCE: Double = 48.0
    }
}

/**
 * This is the LateralTuner OpMode. This tracks the strafe movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the strafe ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 6/26/2025
 */
internal class LateralTuner : OpMode() {
    override fun init() {
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("Pull your robot to the right $DISTANCE inches. Your strafe ticks to inches will be shown on the telemetry.")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.drawCurrent()
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        Tuning.follower!!.update()

        Tuning.telemetryM!!.debug("Distance Moved: " + Tuning.follower!!.pose.y)
        Tuning.telemetryM!!.debug("The multiplier will display what your strafe ticks to inches should be to scale your current distance to $DISTANCE inches.")
        Tuning.telemetryM!!.debug(
            "Multiplier: " + (DISTANCE / (Tuning.follower!!.pose.y / Tuning.follower!!.getPoseTracker()
                .localizer
                .lateralMultiplier))
        )
        Tuning.telemetryM!!.update(telemetry)

        Tuning.drawCurrentAndHistory()
    }

    companion object {
        var DISTANCE: Double = 48.0
    }
}

/**
 * This is the TurnTuner OpMode. This tracks the turning movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current angle in ticks to the specified angle in radians. So, to use this, run the
 * tuner, then pull/push the robot to the specified angle using a protractor or lines on the ground.
 * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
 * multiple trials and average the results. Then, input the multiplier into the turning ticks to
 * radians in your localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
internal class TurnTuner : OpMode() {
    override fun init() {
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("Turn your robot $ANGLE radians. Your turn ticks to inches will be shown on the telemetry.")
        Tuning.telemetryM!!.update(telemetry)

        Tuning.drawCurrent()
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        Tuning.follower!!.update()

        Tuning.telemetryM!!.debug("Total Angle: " + Tuning.follower!!.totalHeading)
        Tuning.telemetryM!!.debug("The multiplier will display what your turn ticks to inches should be to scale your current angle to $ANGLE radians.")
        Tuning.telemetryM!!.debug(
            "Multiplier: " + (ANGLE / (Tuning.follower!!.totalHeading / Tuning.follower!!.getPoseTracker()
                .localizer
                .turningMultiplier))
        )
        Tuning.telemetryM!!.update(telemetry)

        Tuning.drawCurrentAndHistory()
    }

    companion object {
        var ANGLE: Double = 2 * Math.PI
    }
}

/**
 * This is the ForwardVelocityTuner autonomous follower OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
internal class ForwardVelocityTuner : OpMode() {
    private val velocities = ArrayList<Double?>()
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the cache of velocities and the Panels telemetry.  */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("The robot will run at 1 power until it reaches $DISTANCE inches forward.")
        Tuning.telemetryM!!.debug("Make sure you have enough room, since the robot has inertia after cutting power.")
        Tuning.telemetryM!!.debug("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.")
        Tuning.telemetryM!!.debug("Press B on game pad 1 to stop.")
        Tuning.telemetryM!!.debug("pose", Tuning.follower!!.pose)
        Tuning.telemetryM!!.update(telemetry)

        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        var i = 0
        while (i < RECORD_NUMBER) {
            velocities.add(0.0)
            i++
        }
        Tuning.follower!!.startTeleopDrive(true)
        Tuning.follower!!.update()
        end = false
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            Tuning.stopRobot()
            requestOpModeStop()
        }

        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()


        if (!end) {
            if (abs(Tuning.follower!!.pose.x) > DISTANCE) {
                end = true
                Tuning.stopRobot()
            } else {
                Tuning.follower!!.setTeleOpDrive(1.0, 0.0, 0.0, true)
                //double currentVelocity = Math.abs(Tuning.follower!!.getVelocity().getXComponent());
                val currentVelocity: Double =
                    abs(Tuning.follower!!.poseTracker.localizer.velocity.x)
                velocities.add(currentVelocity)
                velocities.removeAt(0)
            }
        } else {
            Tuning.stopRobot()
            var average = 0.0
            for (velocity in velocities) {
                average += velocity!!
            }
            average /= velocities.size.toDouble()
            Tuning.telemetryM!!.debug("Forward Velocity: $average")
            Tuning.telemetryM!!.debug("\n")
            Tuning.telemetryM!!.debug("Press A to set the Forward Velocity temporarily (while robot remains on).")

            for (i in velocities.indices) {
                telemetry.addData(i.toString(), velocities[i])
            }

            Tuning.telemetryM!!.update(telemetry)
            telemetry.update()

            if (gamepad1.aWasPressed()) {
                Tuning.follower!!.setXVelocity(average)
                val message = "XMovement: $average"
                Tuning.changes.add(message)
            }
        }
    }

    companion object {
        var DISTANCE: Double = 48.0
        var RECORD_NUMBER: Double = 10.0
    }
}

/**
 * This is the StrafeVelocityTuner autonomous follower OpMode. This runs the robot right at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
internal class LateralVelocityTuner : OpMode() {
    private val velocities = ArrayList<Double?>()

    private var end = false

    override fun init() {}

    /**
     * This initializes the drive motors as well as the cache of velocities and the Panels
     * Tuning.telemetryM!!.
     */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("The robot will run at 1 power until it reaches $DISTANCE inches to the right.")
        Tuning.telemetryM!!.debug("Make sure you have enough room, since the robot has inertia after cutting power.")
        Tuning.telemetryM!!.debug("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.")
        Tuning.telemetryM!!.debug("Press B on Gamepad 1 to stop.")
        Tuning.telemetryM!!.update(telemetry)

        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run right at full power.  */
    override fun start() {
        var i = 0
        while (i < RECORD_NUMBER) {
            velocities.add(0.0)
            i++
        }
        Tuning.follower!!.startTeleopDrive(true)
        Tuning.follower!!.update()
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run sideways enough, these last velocities recorded are
     * averaged and printed.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            Tuning.stopRobot()
            requestOpModeStop()
        }

        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()

        if (!end) {
            if (abs(Tuning.follower!!.pose.y) > DISTANCE) {
                end = true
                Tuning.stopRobot()
            } else {
                Tuning.follower!!.setTeleOpDrive(0.0, 1.0, 0.0, true)
                val currentVelocity: Double = abs(Tuning.follower!!.velocity.dot(Vector(1.0, Math.PI / 2)))
                velocities.add(currentVelocity)
                velocities.removeAt(0)
            }
        } else {
            Tuning.stopRobot()
            var average = 0.0
            for (velocity in velocities) {
                average += velocity!!
            }
            average /= velocities.size.toDouble()

            Tuning.telemetryM!!.debug("Strafe Velocity: $average")
            Tuning.telemetryM!!.debug("\n")
            Tuning.telemetryM!!.debug("Press A to set the Lateral Velocity temporarily (while robot remains on).")
            Tuning.telemetryM!!.update(telemetry)

            if (gamepad1.aWasPressed()) {
                Tuning.follower!!.setYVelocity(average)
                val message = "YMovement: $average"
                Tuning.changes.add(message)
            }
        }
    }

    companion object {
        var DISTANCE: Double = 48.0
        var RECORD_NUMBER: Double = 10.0
    }
}

/**
 * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
internal class ForwardZeroPowerAccelerationTuner : OpMode() {
    private val accelerations = ArrayList<Double?>()
    private var previousVelocity = 0.0
    private var previousTimeNano: Long = 0

    private var stopping = false
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the Panels Tuning.telemetryM!!.  */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("The robot will run forward until it reaches $VELOCITY inches per second.")
        Tuning.telemetryM!!.debug("Then, it will cut power from the drivetrain and roll to a stop.")
        Tuning.telemetryM!!.debug("Make sure you have enough room.")
        Tuning.telemetryM!!.debug("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.")
        Tuning.telemetryM!!.debug("Press B on Gamepad 1 to stop.")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        Tuning.follower!!.startTeleopDrive(false)
        Tuning.follower!!.update()
        Tuning.follower!!.setTeleOpDrive(1.0, 0.0, 0.0, true)
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            Tuning.stopRobot()
            requestOpModeStop()
        }

        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()

        val heading = Vector(1.0, Tuning.follower!!.pose.heading)
        if (!end) {
            if (!stopping) {
                if (Tuning.follower!!.velocity.dot(heading) > VELOCITY) {
                    previousVelocity = Tuning.follower!!.velocity.dot(heading)
                    previousTimeNano = System.nanoTime()
                    stopping = true
                    Tuning.follower!!.setTeleOpDrive(0.0, 0.0, 0.0, true)
                }
            } else {
                val currentVelocity: Double = Tuning.follower!!.velocity.dot(heading)
                accelerations.add(
                    (currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / 10.0.pow(
                        9.0
                    ))
                )
                previousVelocity = currentVelocity
                previousTimeNano = System.nanoTime()
                if (currentVelocity < Tuning.follower!!.constraints.velocityConstraint) {
                    end = true
                }
            }
        } else {
            var average = 0.0
            for (acceleration in accelerations) {
                average += acceleration!!
            }
            average /= accelerations.size.toDouble()

            Tuning.telemetryM!!.debug("Forward Zero Power Acceleration (Deceleration): $average")
            Tuning.telemetryM!!.debug("\n")
            Tuning.telemetryM!!.debug("Press A to set the Forward Zero Power Acceleration temporarily (while robot remains on).")
            Tuning.telemetryM!!.update(telemetry)

            if (gamepad1.aWasPressed()) {
                Tuning.follower!!.getConstants().setForwardZeroPowerAcceleration(average)
                val message = "Forward Zero Power Acceleration: $average"
                Tuning.changes.add(message)
            }
        }
    }

    companion object {
        var VELOCITY: Double = 30.0
    }
}

/**
 * This is the LateralZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * to the right until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
internal class LateralZeroPowerAccelerationTuner : OpMode() {
    private val accelerations = ArrayList<Double?>()
    private var previousVelocity = 0.0
    private var previousTimeNano: Long = 0
    private var stopping = false
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the Panels telemetry.  */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("The robot will run to the right until it reaches $VELOCITY inches per second.")
        Tuning.telemetryM!!.debug("Then, it will cut power from the drivetrain and roll to a stop.")
        Tuning.telemetryM!!.debug("Make sure you have enough room.")
        Tuning.telemetryM!!.debug("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.")
        Tuning.telemetryM!!.debug("Press B on game pad 1 to stop.")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        Tuning.follower!!.startTeleopDrive(false)
        Tuning.follower!!.update()
        Tuning.follower!!.setTeleOpDrive(0.0, 1.0, 0.0, true)
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            Tuning.stopRobot()
            requestOpModeStop()
        }

        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()

        val heading = Vector(1.0, Tuning.follower!!.pose.heading - Math.PI / 2)
        if (!end) {
            if (!stopping) {
                if (abs(Tuning.follower!!.velocity.dot(heading)) > VELOCITY) {
                    previousVelocity = abs(Tuning.follower!!.velocity.dot(heading))
                    previousTimeNano = System.nanoTime()
                    stopping = true
                    Tuning.follower!!.setTeleOpDrive(0.0, 0.0, 0.0, true)
                }
            } else {
                val currentVelocity: Double = abs(Tuning.follower!!.velocity.dot(heading))
                accelerations.add(
                    (currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / 10.0.pow(
                        9.0
                    ))
                )
                previousVelocity = currentVelocity
                previousTimeNano = System.nanoTime()
                if (currentVelocity < Tuning.follower!!.constraints.velocityConstraint) {
                    end = true
                }
            }
        } else {
            var average = 0.0
            for (acceleration in accelerations) {
                average += acceleration!!
            }
            average /= accelerations.size.toDouble()

            Tuning.telemetryM!!.debug("Lateral Zero Power Acceleration (Deceleration): $average")
            Tuning.telemetryM!!.debug("\n")
            Tuning.telemetryM!!.debug("Press A to set the Lateral Zero Power Acceleration temporarily (while robot remains on).")
            Tuning.telemetryM!!.update(telemetry)

            if (gamepad1.aWasPressed()) {
                Tuning.follower!!.getConstants().setLateralZeroPowerAcceleration(average)
                val message = "Lateral Zero Power Acceleration: $average"
                Tuning.changes.add(message)
            }
        }
    }

    companion object {
        var VELOCITY: Double = 30.0
    }
}

/**
 * This is the Predictive Braking Tuner. It runs the robot forward and backward at various power
 * levels, recording the robot’s velocity and position immediately before braking. The motors are
 * then set to a reverse power, which represents the fastest theoretical braking the robot
 * can achieve. Once the robot comes to a complete stop, the tuner measures the stopping distance.
 * Using the collected data, it generates a velocity-vs-stopping-distance graph and fits a
 * quadratic curve to model the braking behavior.
 *
 * @author Ashay Sarda - 19745 Turtle Walkers
 * @author Jacob Ophoven - 18535 Frozen Code
 * @version 1.0, 12/26/2025
 */
internal class PredictiveBrakingTuner : OpMode() {
    private enum class State {
        START_MOVE,
        WAIT_DRIVE_TIME,
        APPLY_BRAKE,
        WAIT_BRAKE_TIME,
        RECORD,
        DONE
    }

    private class BrakeRecord(var timeMs: Double, var pose: Pose, var velocity: Double)

    private var state = State.START_MOVE

    private val timer: ElapsedTime = ElapsedTime()

    private var iteration = 0

    private var startPosition: Vector? = null
    private var measuredVelocity = 0.0

    private val velocityToBrakingDistance: MutableList<DoubleArray?> = ArrayList()
    private val brakeData: MutableList<BrakeRecord> = ArrayList<BrakeRecord>()

    override fun init() {}

    override fun init_loop() {
        Tuning.telemetryM!!.debug("The robot will move forwards and backwards starting at max speed and slowing down.")
        Tuning.telemetryM!!.debug("Make sure you have enough room. Leave at least 4-5 feet.")
        Tuning.telemetryM!!.debug("After stopping, kFriction and kBraking will be displayed.")
        Tuning.telemetryM!!.debug("Make sure to turn the timer off.")
        Tuning.telemetryM!!.debug("Press B on game pad 1 to stop.")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun start() {
        timer.reset()
        Tuning.follower!!.update()
        Tuning.follower!!.startTeleOpDrive(true)
    }

    @SuppressLint("DefaultLocale")
    override fun loop() {
        Tuning.follower!!.update()

        if (gamepad1.b) {
            Tuning.stopRobot()
            requestOpModeStop()
            return
        }

        val direction = (if (iteration % 2 == 0) 1 else -1).toDouble()

        when (state) {
            State.START_MOVE -> {
                if (iteration >= TEST_POWERS.size) {
                    state = State.DONE
                } else {
                    val currentPower: Double = TEST_POWERS[iteration]
                    Tuning.follower!!.setMaxPower(currentPower)
                    Tuning.follower!!.setTeleOpDrive(direction, 0.0, 0.0, true)

                    timer.reset()
                    state = State.WAIT_DRIVE_TIME
                }
            }

            State.WAIT_DRIVE_TIME -> {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    measuredVelocity = Tuning.follower!!.velocity.magnitude
                    startPosition = Tuning.follower!!.pose.getAsVector()
                    state = State.APPLY_BRAKE
                }
            }

            State.APPLY_BRAKE -> {
                Tuning.follower!!.setTeleOpDrive(BRAKING_POWER * direction, 0.0, 0.0, true)

                timer.reset()
                state = State.WAIT_BRAKE_TIME
            }

            State.WAIT_BRAKE_TIME -> {
                val t = timer.milliseconds()
                val currentPose: Pose = Tuning.follower!!.pose
                val currentVelocity: Double = Tuning.follower!!.velocity.magnitude

                brakeData.add(BrakeRecord(t, currentPose, currentVelocity))

                if (Tuning.follower!!.velocity.dot(
                        Vector(
                            direction,
                            Tuning.follower!!.heading
                        )
                    ) <= 0
                ) {
                    state = State.RECORD
                }
            }

            State.RECORD -> {
                val endPosition: Vector = Tuning.follower!!.pose.getAsVector()
                val brakingDistance = endPosition.minus(startPosition).magnitude

                velocityToBrakingDistance.add(doubleArrayOf(measuredVelocity, brakingDistance))

                Tuning.telemetryM!!.debug(
                    "Test $iteration",
                    String.format(
                        "v=%.3f  d=%.3f", measuredVelocity,
                        brakingDistance
                    )
                )
                Tuning.telemetryM!!.update(telemetry)

                iteration++
                state = State.START_MOVE
            }

            State.DONE -> {
                Tuning.stopRobot()

                val coefficients: DoubleArray = MathFunctions.quadraticFit(velocityToBrakingDistance)

                Tuning.telemetryM!!.debug("Tuning Complete")
                Tuning.telemetryM!!.debug("Braking Profile:")
                Tuning.telemetryM!!.debug("kQuadratic", coefficients[1])
                Tuning.telemetryM!!.debug("kLinear", coefficients[0])
                Tuning.telemetryM!!.update(telemetry)
                Tuning.telemetryM!!.debug("Tuning Complete")
                Tuning.telemetryM!!.debug("Braking Profile:")
                Tuning.telemetryM!!.debug("kQuadraticFriction", coefficients[1])
                Tuning.telemetryM!!.debug("kLinearBraking", coefficients[0])
                for (record in brakeData) {
                    val p = record.pose
                    Tuning.telemetryM!!.debug(
                        String.format(
                            "t=%.0f ms, x=%.2f, y=%.2f, θ=%.2f, v=%.2f",
                            record.timeMs, p.x, p.y,
                            p.heading,
                            record.velocity
                        )
                    )
                }
                Tuning.telemetryM!!.update()
            }
        }
    }

    companion object {
        private val TEST_POWERS = doubleArrayOf(1.0, 1.0, 1.0, 0.9, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2)
        private val BRAKING_POWER = -0.2

        private const val DRIVE_TIME_MS = 1000
    }
}

/**
 * This is the Translational PIDF Tuner OpMode. It will keep the robot in place.
 * The user should push the robot laterally to test the PIDF and adjust the PIDF values accordingly.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class TranslationalTuner : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /** This initializes the Follower and creates the forward and backward Paths.  */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("This will activate the translational PIDF(s)")
        Tuning.telemetryM!!.debug("The robot will try to stay in place while you push it laterally.")
        Tuning.telemetryM!!.debug("You can adjust the PIDF values to tune the robot's translational PIDF(s).")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun start() {
        Tuning.follower!!.deactivateAllPIDFs()
        Tuning.follower!!.activateTranslational()
        forwards = Path(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
        forwards!!.setConstantHeadingInterpolation(0.0)
        backwards = Path(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
        backwards!!.setConstantHeadingInterpolation(0.0)
        Tuning.follower!!.followPath(forwards)
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry  */
    override fun loop() {
        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()

        if (!Tuning.follower!!.isBusy) {
            if (forward) {
                forward = false
                Tuning.follower!!.followPath(backwards)
            } else {
                forward = true
                Tuning.follower!!.followPath(forwards)
            }
        }

        Tuning.telemetryM!!.debug("Push the robot laterally to test the Translational PIDF(s).")
        Tuning.telemetryM!!.update(telemetry)
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Heading PIDF Tuner OpMode. It will keep the robot in place.
 * The user should try to turn the robot to test the PIDF and adjust the PIDF values accordingly.
 * It will try to keep the robot at a constant heading while the user tries to turn it.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class HeadingTuner : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("This will activate the heading PIDF(s).")
        Tuning.telemetryM!!.debug("The robot will try to stay at a constant heading while you try to turn it.")
        Tuning.telemetryM!!.debug("You can adjust the PIDF values to tune the robot's heading PIDF(s).")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun start() {
        Tuning.follower!!.deactivateAllPIDFs()
        Tuning.follower!!.activateHeading()
        forwards = Path(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
        forwards!!.setConstantHeadingInterpolation(0.0)
        backwards = Path(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
        backwards!!.setConstantHeadingInterpolation(0.0)
        Tuning.follower!!.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()

        if (!Tuning.follower!!.isBusy) {
            if (forward) {
                forward = false
                Tuning.follower!!.followPath(backwards)
            } else {
                forward = true
                Tuning.follower!!.followPath(forwards)
            }
        }

        Tuning.telemetryM!!.debug("Turn the robot manually to test the Heading PIDF(s).")
        Tuning.telemetryM!!.update(telemetry)
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Drive PIDF Tuner OpMode. It will run the robot in a straight line going forward and back.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class DriveTuner : OpMode() {
    private var forward = true

    private var forwards: PathChain? = null
    private var backwards: PathChain? = null

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("This will run the robot in a straight line going " + DISTANCE + "inches forward.")
        Tuning.telemetryM!!.debug("The robot will go forward and backward continuously along the path.")
        Tuning.telemetryM!!.debug("Make sure you have enough room.")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun start() {
        Tuning.follower!!.deactivateAllPIDFs()
        Tuning.follower!!.activateDrive()

        forwards = Tuning.follower!!.pathBuilder()
            .setGlobalDeceleration()
            .addPath(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
            .setConstantHeadingInterpolation(0.0)
            .build()

        backwards = Tuning.follower!!.pathBuilder()
            .setGlobalDeceleration()
            .addPath(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
            .setConstantHeadingInterpolation(0.0)
            .build()

        Tuning.follower!!.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()

        if (!Tuning.follower!!.isBusy) {
            if (forward) {
                forward = false
                Tuning.follower!!.followPath(backwards)
            } else {
                forward = true
                Tuning.follower!!.followPath(forwards)
            }
        }

        Tuning.telemetryM!!.debug("Driving forward?: $forward")
        Tuning.telemetryM!!.update(telemetry)
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Line Test Tuner OpMode. It will drive the robot forward and back
 * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class Line : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /** This initializes the Follower and creates the forward and backward Paths.  */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("This will activate all the PIDF(s)")
        Tuning.telemetryM!!.debug("The robot will go forward and backward continuously along the path while correcting.")
        Tuning.telemetryM!!.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun start() {
        Tuning.follower!!.activateAllPIDFs()
        forwards = Path(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
        forwards!!.setConstantHeadingInterpolation(0.0)
        backwards = Path(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
        backwards!!.setConstantHeadingInterpolation(0.0)
        Tuning.follower!!.followPath(forwards)
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry  */
    override fun loop() {
        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()

        if (!Tuning.follower!!.isBusy) {
            if (forward) {
                forward = false
                Tuning.follower!!.followPath(backwards)
            } else {
                forward = true
                Tuning.follower!!.followPath(forwards)
            }
        }

        Tuning.telemetryM!!.debug("Driving Forward?: $forward")
        Tuning.telemetryM!!.update(telemetry)
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * @author Jacob Ophoven - 18535 Frozen Code
 */
internal class Line90DegreeTurn : OpMode() {
    override fun init() {}

    /** This initializes the Follower and creates the forward and backward Paths.  */
    override fun init_loop() {
        Tuning.telemetryM!!.debug(
            "The robot will go forward 48 inches and then sideways to " +
                    "the left 24 inches."
        )
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun start() {
        Tuning.follower!!.activateAllPIDFs()
        val forwards = Path(BezierLine(Pose(0.0, 0.0), Pose(48.0, 0.0)))
        forwards.setConstantHeadingInterpolation(0.0)
        val sideways = Path(
            BezierLine(
                Pose(48.0, 0.0), Pose(
                    48.0,
                    24.0
                )
            )
        )
        sideways.setConstantHeadingInterpolation(0.0)
        Tuning.follower!!.followPath(PathChain(forwards, sideways))
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry  */
    override fun loop() {
        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()

        if (!Tuning.follower!!.isBusy) {
            Tuning.stopRobot()
        }

        Tuning.telemetryM!!.update(telemetry)
    }
}

/**
 * This is the Centripetal Tuner OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
internal class CentripetalTuner : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths.
     * Additionally, this initializes the Panels telemetry.
     */
    override fun init_loop() {
        Tuning.telemetryM!!.debug("This will run the robot in a curve going $DISTANCE inches to the left and the same number of inches forward.")
        Tuning.telemetryM!!.debug("The robot will go continuously along the path.")
        Tuning.telemetryM!!.debug("Make sure you have enough room.")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun start() {
        Tuning.follower!!.activateAllPIDFs()
        forwards = Path(BezierCurve(Pose(), Pose(abs(DISTANCE), 0.0), Pose(abs(DISTANCE), DISTANCE)))
        backwards = Path(BezierCurve(Pose(abs(DISTANCE), DISTANCE), Pose(abs(DISTANCE), 0.0), Pose(0.0, 0.0)))

        backwards!!.setTangentHeadingInterpolation()
        backwards!!.reverseHeadingInterpolation()

        Tuning.follower!!.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()
        if (!Tuning.follower!!.isBusy) {
            if (forward) {
                forward = false
                Tuning.follower!!.followPath(backwards)
            } else {
                forward = true
                Tuning.follower!!.followPath(forwards)
            }
        }

        Tuning.telemetryM!!.debug("Driving away from the origin along the curve?: $forward")
        Tuning.telemetryM!!.update(telemetry)
    }

    companion object {
        var DISTANCE: Double = 20.0
    }
}

/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
internal class Triangle : OpMode() {
    private val startPose = Pose(0.0, 0.0, Math.toRadians(0.0))
    private val interPose = Pose(24.0, -24.0, Math.toRadians(90.0))
    private val endPose = Pose(24.0, 24.0, Math.toRadians(45.0))

    private var triangle: PathChain? = null

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()

        if (Tuning.follower!!.atParametricEnd()) {
            Tuning.follower!!.followPath(triangle, true)
        }
    }

    override fun init() {}

    override fun init_loop() {
        Tuning.telemetryM!!.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.")
        Tuning.telemetryM!!.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    /** Creates the PathChain for the "triangle". */
    override fun start() {
        Tuning.follower!!.setStartingPose(startPose)

        triangle = Tuning.follower!!.pathBuilder()
            .addPath(BezierLine(startPose, interPose))
            .setLinearHeadingInterpolation(startPose.heading, interPose.heading)
            .addPath(BezierLine(interPose, endPose))
            .setLinearHeadingInterpolation(interPose.heading, endPose.heading)
            .addPath(BezierLine(endPose, startPose))
            .setLinearHeadingInterpolation(endPose.heading, startPose.heading)
            .build()

        Tuning.follower!!.followPath(triangle)
    }
}

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class Circle : OpMode() {
    private var circle: PathChain? = null

    override fun start() {
        circle = Tuning.follower!!.pathBuilder()
            .addPath(BezierCurve(Pose(0.0, 0.0), Pose(RADIUS, 0.0), Pose(RADIUS, RADIUS)))
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .addPath(BezierCurve(Pose(RADIUS, RADIUS), Pose(RADIUS, 2 * RADIUS), Pose(0.0, 2 * RADIUS)))
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .addPath(BezierCurve(Pose(0.0, 2 * RADIUS), Pose(-RADIUS, 2 * RADIUS), Pose(-RADIUS, RADIUS)))
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .addPath(BezierCurve(Pose(-RADIUS, RADIUS), Pose(-RADIUS, 0.0), Pose(0.0, 0.0)))
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .build()
        Tuning.follower!!.followPath(circle)
    }

    override fun init_loop() {
        Tuning.telemetryM!!.debug("This will run in a roughly circular shape of radius $RADIUS, starting on the right-most edge. ")
        Tuning.telemetryM!!.debug("So, make sure you have enough space to the left, front, and back to run the OpMode.")
        Tuning.telemetryM!!.debug("It will also continuously face the center of the circle to test your heading and centripetal correction.")
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun init() {}

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    override fun loop() {
        Tuning.follower!!.update()
        Tuning.drawCurrentAndHistory()

        if (Tuning.follower!!.atParametricEnd()) {
            Tuning.follower!!.followPath(circle)
        }
    }

    companion object {
        var RADIUS: Double = 10.0
    }
}

/**
 * Tuning OpMode to get the min and max encoder values for swerve pods
 * @author Kabir Goyal
 */
internal class AnalogMinMaxTuner : OpMode() {
    //populate the below with your names for the servos and encoders
    var encoderNames: Array<String?> =
        arrayOf("leftFrontEncoder", "rightFrontEncoder", "leftBackEncoder", "rightBackEncoder")
    var encoders: Array<AnalogInput?> = arrayOfNulls(encoderNames.size)
    var minVoltages: DoubleArray = DoubleArray(encoderNames.size)
    var maxVoltages: DoubleArray = DoubleArray(encoderNames.size)

    var lynxModules: MutableList<LynxModule>? = null //js to improve loop times a bit yk

    override fun start() {
    }

    override fun init_loop() {
        Tuning.telemetryM!!.debug(
            "Press START. Then, Spin each pod slowly for 4 to 5 full rotations.\n" +
                    "The OpMode will keep track of the min and max voltages seen so far and print them to telemetry."
        )
        Tuning.telemetryM!!.update(telemetry)
    }

    override fun init() {
        lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (hub in lynxModules!!) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        }

        for (i in encoders.indices) {
            encoders[i] = hardwareMap.get(AnalogInput::class.java, encoderNames[i])
            minVoltages[i] = 5.0 //bigger value than should ever be read
        }
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    override fun loop() {
        for (hub in lynxModules!!) {
            hub.clearBulkCache()
        }

        Tuning.telemetryM!!.debug(
            "Spin each pod slowly for 4 to 5 full rotations.\n" +
                    "The OpMode will keep track of the min and max voltages seen so far and print them to telemetry.\n\n"
        )

        for (i in encoders.indices) {
            val currentVoltage = encoders[i]!!.voltage
            minVoltages[i] = min(minVoltages[i], currentVoltage)
            maxVoltages[i] = max(maxVoltages[i], currentVoltage)
            Tuning.telemetryM!!.addData(encoderNames[i] + "min value:", minVoltages[i])
            Tuning.telemetryM!!.addData(encoderNames[i] + "max value:", maxVoltages[i])
            Tuning.telemetryM!!.addLine("")
        }

        Tuning.telemetryM!!.update()
    }
}

/**
 * This is the SwerveOffsetsTest
 * You should use this to check how good your swerve angle offsets are and if your motor directions are correct
 * @author Kabir Goyal
 */
internal class SwerveOffsetsTest : OpMode() {
    var debugStringEnabled: Boolean = false

    override fun init() {}

    /** This initializes the PoseUpdater, the drive motors, and the Panels telemetry.  */
    override fun init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled
        }


        Tuning.telemetryM!!.debug(
            "This OpMode will run all four swerve pods in the direction they think is forward"
                    + "\nensure your bot is not on the ground while running"
        )
        Tuning.telemetryM!!.debug(
            "Drivetrain debug string " + ((if (debugStringEnabled) "enabled" else "disabled")) +
                    " (press gamepad a to toggle)"
        )
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun start() {
        Tuning.follower!!.startTeleopDrive()
        Tuning.follower!!.update()
    }

    /**
     * This updates the robot's pose estimate, the simple drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    override fun loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled
        }

        Tuning.follower!!.setTeleOpDrive(0.25, 0.0, 0.0, true)
        Tuning.follower!!.update()

        if (debugStringEnabled) {
            Tuning.telemetryM!!.debug(
                "Drivetrain Debug String:\n" +
                        Tuning.follower!!.getDrivetrain().debugString()
            )
        }
        Tuning.telemetryM!!.update(telemetry)

        Tuning.drawCurrentAndHistory()
    }
}

/**
 * This is the SwerveTurnTest
 * You should use this to check your encoder directions and x/y pod offsets
 * @author Kabir Goyal
 */
internal class SwerveTurnTest : OpMode() {
    var debugStringEnabled: Boolean = false

    override fun init() {}

    /** This initializes the PoseUpdater, the drive motors, and the Panels telemetry.  */
    override fun init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled
        }


        Tuning.telemetryM!!.debug(
            "This OpMode will run all four swerve pods in their turning direction (perpendicular to the center of the robot) "
                    + "\nrun this once off the ground to check servo directions and motor directions before testing on the ground"
        )
        Tuning.telemetryM!!.debug(
            "Drivetrain debug string " + ((if (debugStringEnabled) "enabled" else "disabled")) +
                    " (press gamepad a to toggle)"
        )
        Tuning.telemetryM!!.update(telemetry)
        Tuning.follower!!.update()
        Tuning.drawCurrent()
    }

    override fun start() {
        Tuning.follower!!.startTeleopDrive()
        Tuning.follower!!.update()
    }

    /**
     * This updates the robot's pose estimate, the simple drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    override fun loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled
        }

        Tuning.follower!!.setTeleOpDrive(0.0, 0.0, 0.25, true)
        Tuning.follower!!.update()

        if (debugStringEnabled) {
            Tuning.telemetryM!!.debug(
                "Drivetrain Debug String:\n" +
                        Tuning.follower!!.getDrivetrain().debugString()
            )
        }
        Tuning.telemetryM!!.update(telemetry)

        Tuning.drawCurrentAndHistory()
    }
}


/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
internal object Drawing {
    val ROBOT_RADIUS: Double = 9.0 // woah
    private val panelsField: FieldManager = PanelsField.field

    private val robotLook = Style(
        "", "#3F51B5", 0.0
    )
    private val historyLook = Style(
        "", "#4CAF50", 0.0
    )

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    fun init() {
        panelsField.setOffsets(PanelsField.presets.PEDRO_PATHING)
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    fun drawDebug(follower: Follower) {
        if (follower.currentPath != null) {
            drawPath(follower.currentPath, robotLook)
            val closestPoint: Pose =
                follower.getPointFromPath(follower.currentPath.closestPointTValue)
            drawRobot(
                Pose(
                    closestPoint.x,
                    closestPoint.y,
                    follower.currentPath
                        .getHeadingGoal(follower.currentPath.closestPointTValue)
                ), robotLook
            )
        }
        drawPoseHistory(follower.poseHistory, historyLook)
        drawRobot(follower.pose, historyLook)

        sendPacket()
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    @JvmOverloads
    fun drawRobot(pose: Pose?, style: Style = robotLook) {
        if (pose == null || (pose.x).isNaN() || (pose.y).isNaN() || (pose.heading).isNaN()) {
            return
        }

        panelsField.setStyle(style)
        panelsField.moveCursor(pose.x, pose.y)
        panelsField.circle(ROBOT_RADIUS)

        val v = pose.headingAsUnitVector
        v.magnitude *= ROBOT_RADIUS
        val x1 = pose.x + v.xComponent / 2
        val y1 = pose.y + v.yComponent / 2
        val x2 = pose.x + v.xComponent
        val y2 = pose.y + v.yComponent

        panelsField.setStyle(style)
        panelsField.moveCursor(x1, y1)
        panelsField.line(x2, y2)
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    fun drawPath(path: Path, style: Style) {
        val points = path.panelsDrawingPoints

        for (i in points[0]!!.indices) {
            for (j in points.indices) {
                if ((points[j]!![i]).isNaN()) {
                    points[j]!![i] = 0.0
                }
            }
        }

        panelsField.setStyle(style)
        panelsField.moveCursor(points[0]!![0], points[0]!![1])
        panelsField.line(points[1]!![0], points[1]!![1])
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    fun drawPath(pathChain: PathChain, style: Style) {
        for (i in 0..<pathChain.size()) {
            drawPath(pathChain.getPath(i), style)
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    @JvmOverloads
    fun drawPoseHistory(poseTracker: PoseHistory, style: Style = historyLook) {
        panelsField.setStyle(style)

        val size: Int = poseTracker.xPositionsArray.size
        for (i in 0..<size - 1) {
            panelsField.moveCursor(poseTracker.xPositionsArray[i], poseTracker.yPositionsArray[i])
            panelsField.line(poseTracker.xPositionsArray[i + 1], poseTracker.yPositionsArray[i + 1])
        }
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    fun sendPacket() {
        panelsField.update()
    }
}
