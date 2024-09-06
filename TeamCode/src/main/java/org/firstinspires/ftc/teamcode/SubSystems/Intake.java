package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Intake extends SubsystemBase{

    MMRobot mmRobot = MMRobot.getInstance();

    CuttleMotor motor;
    public Intake() {
        motor = mmRobot.mmSystems.controlHub.getMotor(Configuration.INTAKE);
    }

    public void setPower(double power){motor.setPower(power);}

}
