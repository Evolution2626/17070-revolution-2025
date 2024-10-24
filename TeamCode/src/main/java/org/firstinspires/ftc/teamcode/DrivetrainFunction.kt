package org.firstinspires.ftc.teamcode

import kotlin.math.abs

class DrivetrainFunction {

    companion object {
        @JvmStatic
        fun calculateTicks(Cm: Double): Double {

            val COUNTS_PER_MOTOR_REV = 536.6

            val WHEEL_DIAMETER_CM = 10.0

            val COUNTS_PER_Cm = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * 3.1415)
            return COUNTS_PER_Cm * Cm
        }

        @JvmStatic
        fun calculatePower(targetPos: Double, currentPos: Double): Double {
            return if (abs(targetPos - currentPos) > 50) {
                if (abs(currentPos - targetPos) < 100) 0.25
                else if (abs(currentPos - targetPos) < 500) 0.50
                else if (abs(currentPos - targetPos) < 1000) 0.75
                else 1.0
            } else 0.0

        }
    }
}
