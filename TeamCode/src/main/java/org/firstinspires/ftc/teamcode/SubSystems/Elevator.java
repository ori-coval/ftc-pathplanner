package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Elevator extends SubsystemBase {

    MMRobot robot = MMRobot.getInstance();

    CuttleMotor motorRight;
    CuttleMotor motorLeft;
    CuttleEncoder motorLeftEncoder;

    public Elevator() {
        this.motorRight = new CuttleMotor(robot.mmSystems.expansionHub,(Configuration.ELEVATOR_RIGHT));
        this.motorLeft = new CuttleMotor(robot.mmSystems.expansionHub,(Configuration.ELEVATOR_LEFT));
        this.motorLeftEncoder = new CuttleEncoder(robot.mmSystems.expansionHub,Configuration.ELEVATOR_ENCODER,1);
    }

    public void setPower(double power){
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public double getHight(){
        return motorLeftEncoder.getCounts();
    }

}
