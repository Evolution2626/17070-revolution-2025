package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DigitalChannel

class ElevatorFunction {
    companion object {
        @JvmStatic
        fun moveElevator(
            power: Double,
            elevatorIn: DigitalChannel,
            elevatorOut: DigitalChannel
        ): Double {
            return if (elevatorOut.state && power <= 0) power
            else if (elevatorIn.state && power >= 0) power
            else 0.0
        }
    }
}