package org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.tasks

import com.roboctopi.cuttlefish.queue.Task
import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMServo

class ServoPresetTask(val servo: MMServo, val preset: Int): Task
{
    override fun loop(): Boolean
    {
        servo.goToPreset(preset);
        return true;
    }
}