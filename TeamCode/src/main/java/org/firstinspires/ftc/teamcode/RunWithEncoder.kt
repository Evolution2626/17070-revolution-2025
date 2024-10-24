package org.firstinspires.ftc.teamcode

import kotlin.math.abs

class RunWithEncoder() {
    fun calculatePower(targetPos:Double, currentPos:Double): Double {
        if(abs(targetPos - currentPos) > 50) {
            if (abs(currentPos - targetPos) < 100) {
                return 0.25
            }
            if(abs(currentPos - targetPos) < 500) {
                return 0.50
            }
            return if(abs(currentPos - targetPos) < 1000) {
                0.75
            } else {
                1.0
            }

        } else {
            return 0.0
        }
    }
    fun calculateTicks(Cm:Double): Double {

        val COUNTS_PER_MOTOR_REV = 536.6

        val WHEEL_DIAMETER_CM = 10.0

        val COUNTS_PER_Cm = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * 3.1415)
        return COUNTS_PER_Cm * Cm
    }



}