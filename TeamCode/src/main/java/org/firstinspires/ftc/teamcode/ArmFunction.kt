package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.AnalogInput

class ArmFunction {
    companion object {
        @JvmStatic
        fun moveArm(power: Double, armSensor: AnalogInput): Double {
            return if (power >= 0 && armSensor.voltage < 0.75) power
            else if (power >= 0 && armSensor.voltage > 2.55) power
            else 0.0
        }
    }
}