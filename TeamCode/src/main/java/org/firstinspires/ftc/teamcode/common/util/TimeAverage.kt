package org.firstinspires.ftc.teamcode.common.util

class TimeAverage(val duration: () -> Double) {
    val queue: ArrayDeque<Pair<Long, Double>> = ArrayDeque()

    val average: Double
        get() = queue.map { it.second }.average()

    fun update(value: Double) {
        val current = System.nanoTime()
        queue.add(current to value)

        val threshold = current - duration()
        queue.removeIf { it.first <= threshold }
    }
}