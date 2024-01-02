package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret extends SubsystemBase {
    CRServo turretServoA;
    CRServo turretServoB;
    DcMotor encoder;
    final double offset = 0;
    private PIDController pidController = new PIDController(1.0/180,0,0);

    public Turret(CRServo turretMotorA, CRServo turretMotorB) {
        this.turretServoA = turretMotorA;
        this.turretServoB = turretMotorB;
//        this.encoder = encoder;
    }
    public void setPower (double power) {
        turretServoA.setPower(power);
        turretServoB.setPower(power);
    }
    public double getEncoderValue(){
        return encoder.getCurrentPosition();
    }

    public void stop(){
        setPower(0);
    }

    public PIDController getPidController() {
        return pidController;
    }
}

