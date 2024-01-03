package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret extends SubsystemBase {
    CRServo turretServoA;
    CRServo turretServoB;
    AnalogInput turretEncoder;
    final double offset = 0;
    private PIDController pidController = new PIDController(1.0/180,0,0);

    public Turret(CRServo turretMotorA, CRServo turretMotorB, AnalogInput turretEncoder) {
        this.turretServoA = turretMotorA;
        this.turretServoB = turretMotorB;
        this.turretEncoder = turretEncoder;

    }
    public void setPower (double power) {
        turretServoA.setPower(power);
        turretServoB.setPower(power);
    }
    public double getEncoderValue(){
        return turretEncoder.getVoltage() ;
    }

    public void stop(){
        setPower(0);
    }

    public PIDController getPidController() {
        return pidController;
    }
}

