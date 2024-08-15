package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.MMMotor;

import java.util.ArrayList;

public class MotorInterlacer {
    public ArrayList<MMMotor[]> groups = new ArrayList<>();

    public void addMotorGroup(MMMotor... motors)
    {
        groups.add(motors);
        for(int j = 0; j < motors.length; j++)
        {
            motors[j].interlaced = true;
        }
    }
    private int i = 0;
    public void loop()
    {
        MMMotor[] group = groups.get(i);
        for(int j = 0; j < group.length; j++)
        {
            group[j].sendPower();
        }
        i++;
        i %= group.length;
    }

}
