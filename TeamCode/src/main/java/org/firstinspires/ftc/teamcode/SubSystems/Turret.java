package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret extends SubsystemBase {
    private CRServo turretServoA;
    private CRServo turretServoB;
    private DcMotor encoder;
    private final double OFFSET = 0;
    private final double TICKS_PER_REV = 1;
    private PIDController pidController = new PIDController(1.0/180,0,0);

    public Turret(CRServo turretMotorA, CRServo turretMotorB, DcMotor encoder){
        this.turretServoA = turretMotorA;
        this.turretServoB = turretMotorB;
        this.encoder = encoder;
    }
    public void setPower (double power) {
        turretServoA.setPower(power);
        turretServoB.setPower(power);
    }
    public double getAngle(){
        return encoder.getCurrentPosition()/TICKS_PER_REV * 360;
    }

    public void stop(){
        setPower(0);
    }

    public PIDController getPidController() {
        return pidController;
    }
}

