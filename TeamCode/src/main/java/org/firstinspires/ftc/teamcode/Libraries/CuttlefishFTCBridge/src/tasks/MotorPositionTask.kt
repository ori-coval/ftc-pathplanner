package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.tasks

import com.roboctopi.cuttlefish.queue.Task
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor

class MotorPositionTask(val position:Double, val motor: CuttleMotor, val wait:Boolean = false, val epsilon: Float = 0.05f):Task{

    override fun loop(): Boolean {
        motor.setPosition(position);
        if(wait)
        {
            return motor.positionController.isAtGoal(epsilon);
        }
        else
        {
            return true
        }
    }
}