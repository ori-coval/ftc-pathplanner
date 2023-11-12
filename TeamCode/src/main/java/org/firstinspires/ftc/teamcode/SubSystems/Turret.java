package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret extends SubsystemBase {
    CRServo turretServoA;
    CRServo turretServoB;
    DcMotorEx fL;
    
    final double offset = 0;
    public Turret(CRServo turretMotorA, CRServo turretMotorB, AnalogInput encoder){
        this.turretServoA = turretMotorA;
        this.turretServoB = turretMotorB;
    }
    public void setPower (double power) {
        turretServoA.setPower(power);
        turretServoB.setPower(power);
    }
    public double getEncoderValue(){
        return fL.getCurrentPosition();
    }

    public void stop(){
        setPower(0);
    }

}

