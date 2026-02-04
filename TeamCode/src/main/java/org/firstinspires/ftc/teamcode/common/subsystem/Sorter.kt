//package org.firstinspires.ftc.teamcode.common.subsystem
//
//import com.bylazar.configurables.annotations.Configurable
//import com.millburnx.cmdx.Command
//import com.millburnx.cmdxpedro.util.WaitFor
//import com.qualcomm.hardware.rev.RevColorSensorV3
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor
//import org.firstinspires.ftc.teamcode.opmode.OpMode
//
//@Configurable
//class Sorter(opMode: OpMode) : Subsystem("Sorter") {
//    val c0 = opMode.hardwareMap.colorSensor["c0"]
//    val c1 = opMode.hardwareMap["c1"] as RevColorSensorV3
//    val c2 = opMode.hardwareMap["c2"] as Normal
//    izedColorSensor
//
//    override val run: suspend Command.() -> Unit = {
//        with(opMode) {
//            WaitFor { isStarted || isStopRequested }
//            while (!isStopRequested) {
//                c1.argb()
//                c1.normalizedColors.let {
//                    it.toColor()
//                }
//                c1.close()
//                c1.rawLightDetected
//                c1.rawLightDetectedMax
//                c1.rawOptical()
//                c1.lightDetected
//                c1.gain
//                c1.enableLed(true)
//                c1.isLightOn
//
//                c0.argb()
//                c0.enableLed(true)
//
//                c2.normalizedColors
//                c2.gain
//                sync()
//            }
//        }
//    }
//
//    enum class State {
//        Empty,
//        Green,
//        Purple
//    }
//
//    companion object {
//        @JvmField
//        var c0p = 67
//
//        @JvmField
//        var c0g = 87
//
//        @JvmField
//        var c1p = 34
//
//        @JvmField
//        var c1g = 35
//
//        @JvmField
//        var c2p = 34
//
//        @JvmField
//        var c2g = 39
//
//        @JvmField
//        var c3p = 23
//
//        @JvmField
//        var c3g = 24
//
//        @JvmField
//        var c4p = 34
//
//        @JvmField
//        var c4g = 40
//
//        @JvmField
//        var c5p = 13
//
//        @JvmField
//        var c5g = 14
//    }
//}