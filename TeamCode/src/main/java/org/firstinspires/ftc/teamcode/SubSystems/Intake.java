package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Intake extends SubsystemBase{

    MMRobot mmRobot = MMRobot.getInstance();

    CRServo servo;
    public Intake() {
        servo = MMRobot.getInstance().mmSystems.hardwareMap.crservo.get("intakeRoller");

    }

    public void setPower(double power){servo.setPower(power);}


}
