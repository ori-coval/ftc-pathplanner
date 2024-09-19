package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Crab extends SubsystemBase {

    MMRobot robot = MMRobot.getInstance();

    CuttleServo servo;
    final double CLOSE = 0.2;
    final double OPEN = 0.7;

    public Crab(){
        servo = new CuttleServo(robot.mmSystems.controlHub,Configuration.CRAB_SCORING);
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }

    public double getPosition(){
        return servo.getPosition();
    }

}
