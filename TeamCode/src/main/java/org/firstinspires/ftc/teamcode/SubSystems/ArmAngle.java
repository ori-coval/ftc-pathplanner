package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class ArmAngle extends SubsystemBase {

    MMRobot mmRobot = MMRobot.getInstance();

    CuttleServo servoRight;
    CuttleServo servoLeft;

    public ArmAngle(){
        servoRight = new CuttleServo(mmRobot.mmSystems.controlHub, Configuration.ARM_ANGLE_RIGHT);
        servoLeft = new CuttleServo(mmRobot.mmSystems.controlHub, Configuration.ARM_ANGEL_LEFT);
    }

    public void setPosition(double position){
        servoLeft.setPosition(position);
        servoRight.setPosition(position);
    }


}
