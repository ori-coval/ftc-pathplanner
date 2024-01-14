package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends SubsystemBase {
    private CRServo turretServoA;
    private CRServo turretServoB;
    private DcMotor turretEncoder;
    private final double OFFSET = 0;
    private final double TICKS_PER_REV = 8192;
    private PIDController pidController = new PIDController(1.0/180,0,0);

    public Turret(CRServo turretMotorA, CRServo turretMotorB, DcMotor turretEncoder) {
        this.turretServoA = turretMotorA;
        this.turretServoB = turretMotorB;
        this.turretEncoder = turretEncoder;

    }
    public void setPower (double power) {
        turretServoA.setPower(power);
        turretServoB.setPower(power);
    }
    public double getAngle(){
        return turretEncoder.getCurrentPosition()/TICKS_PER_REV * 360;
    }

    public void stop(){
        setPower(0);
    }

    public PIDController getPidController() {
        return pidController;
    }

}

