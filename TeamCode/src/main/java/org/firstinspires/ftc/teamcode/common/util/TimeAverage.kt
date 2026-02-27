package org.firstinspires.ftc.teamcode.common.util

class TimeAverage(val duration: () -> Double) {
    val queue: ArrayDeque<Pair<Long, Double>> = ArrayDeque()

    private var _average: Double = 0.0

    val average: Double
        get() = _average

    fun update(value: Double) {
        val current = System.currentTimeMillis()
        queue.add(current to value)

        val threshold = current - duration()
        queue.removeIf { it.first <= threshold }

        _average = queue.map { it.second }.average()
    }
}