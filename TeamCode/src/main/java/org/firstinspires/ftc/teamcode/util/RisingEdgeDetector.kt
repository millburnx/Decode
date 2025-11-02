package org.firstinspires.ftc.teamcode.util

import com.millburnx.cmdx.Command

class RisingEdgeDetector(
    val state: () -> Boolean,
    val name: String = "Rising Edge Detector",
    val runnable: () -> Unit
) {
    var previousState: Boolean = false

    val command = Command(this.name) {
        val currentState = state()
        if (currentState && !previousState) {
            runnable()
        }
        previousState = currentState
    }
}