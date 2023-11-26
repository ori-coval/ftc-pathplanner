package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret extends SubsystemBase {
    CRServo turretServoA;
    CRServo turretServoB;
<<<<<<< HEAD
    DcMotorEx fL;
    
    final double offset = 0;
    public Turret(CRServo turretMotorA, CRServo turretMotorB, AnalogInput encoder){
        this.turretServoA = turretMotorA;
        this.turretServoB = turretMotorB;
=======
    DcMotor encoder;
    final double offset = 0;
    public Turret(CRServo turretMotorA, CRServo turretMotorB, DcMotor encoder){
        this.turretServoA = turretMotorA;
        this.turretServoB = turretMotorB;
        this.encoder = encoder;
>>>>>>> master
    }
    public void setPower (double power) {
        turretServoA.setPower(power);
        turretServoB.setPower(power);
    }
    public double getEncoderValue(){
<<<<<<< HEAD
        return fL.getCurrentPosition();
=======
        return encoder.getCurrentPosition();
>>>>>>> master
    }

    public void stop(){
        setPower(0);
    }

}

