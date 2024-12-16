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

            return if ((targetPos - currentPos) >= 5) {
                if (abs(currentPos - targetPos) > 300) 0.5
                else if (abs(currentPos - targetPos) > 100) 0.4
                else if (abs(currentPos - targetPos) > 50) 0.3
                else if (abs(currentPos - targetPos) > 5) 0.2

                else 0.8
            } else if ((targetPos - currentPos) <= -5 ) {
                if (abs(currentPos - targetPos) > 200) -0.5
                else if (abs(currentPos - targetPos) > 100) -0.4
                else if (abs(currentPos - targetPos) > 50) -0.3
                else if (abs(currentPos - targetPos) >5) -0.2

                else -0.8
            } else 0.0
        }

    }
}
