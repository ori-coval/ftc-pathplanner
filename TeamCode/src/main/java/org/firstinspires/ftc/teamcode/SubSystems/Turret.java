package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class Turret extends SubsystemBase {
    CRServo turretMotorA;
    CRServo turretMotorB;
    AnalogInput encoder;
    final double offset = 0;
    public Turret(CRServo turretMotorA, CRServo turretMotorB, AnalogInput encoder){
        this.turretMotorA = turretMotorA;
        this.turretMotorB = turretMotorB;
        this.encoder = encoder;
    }
    public void setPower (double power) {
        turretMotorA.setPower(power);
        turretMotorB.setPower(power);
    }
    public double getEncoderValue(){
        return encoder.getVoltage()-offset;
    }

    public void stop(){
        setPower(0);
    }

}

