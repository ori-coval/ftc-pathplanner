package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.tasks

import com.roboctopi.cuttlefish.queue.Task
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.MMMotor

class MotorPositionTask(val position:Double, val motor: MMMotor, val wait:Boolean = false, val epsilon: Float = 0.05f):Task{

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