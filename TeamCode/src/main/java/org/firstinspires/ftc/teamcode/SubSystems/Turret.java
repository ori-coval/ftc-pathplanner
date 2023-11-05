package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret extends SubsystemBase {
    CRServo turretMotorA;
    CRServo turretMotorB;
    AnalogInput encoder;
    public Turret(CRServo turretMotorA, CRServo turretMotorB, AnalogInput encoder){
        this.turretMotorA = turretMotorA;
        this.turretMotorB = turretMotorB;
        this.encoder = encoder;
    }
    public void setPower (double power) {
        turretMotorA.setPower(power);
        turretMotorB.setPower(power);
    }
    public double getEncoderVoltage(){
        return encoder.getVoltage();
    }

    public void stop(){
        setPower(0);
    }

}

