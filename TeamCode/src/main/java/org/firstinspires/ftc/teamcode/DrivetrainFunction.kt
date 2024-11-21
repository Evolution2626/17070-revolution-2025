package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.ColorSensor
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

            return if ((targetPos - currentPos) >= 700) {
                if (abs(currentPos - targetPos) < 700) 0.2
                else if (abs(currentPos - targetPos) < 1000) 0.40
                else if (abs(currentPos - targetPos) < 1500) 0.6
                else 1.0
            } else if ((targetPos - currentPos) >= -700) {
                if (abs(currentPos - targetPos) < 700) -0.25
                else if (abs(currentPos - targetPos) < 1000) -0.4
                else if (abs(currentPos - targetPos) < 1500) -0.6
                else -1.0
            } else 0.0
        }

    }
}
