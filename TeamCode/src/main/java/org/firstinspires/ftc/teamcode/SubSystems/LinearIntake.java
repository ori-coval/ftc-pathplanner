package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class LinearIntake extends SubsystemBase {

    MMRobot mmRobot = MMRobot.getInstance();
    CuttleServo servo;

    final double OPEN = 0;
    final double CLOSE = 0.25;

    public LinearIntake(){
        servo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub,1 );

    }




}
