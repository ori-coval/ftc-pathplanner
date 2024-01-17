package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends SubsystemBase {
    private CRServo turretServoA;
    private CRServo turretServoB;
    private DcMotor turretEncoder;
    private final double OFFSET = 0;
    private final double TICKS_PER_REV = 8192;
    private final double GEAR_RATIO = 21.0/95;
    private PIDController pidController = new PIDController(0.003,0,0);

    public Turret(CRServo turretMotorA, CRServo turretMotorB, DcMotor turretEncoder) {
        this.turretServoA = turretMotorA;
        this.turretServoB = turretMotorB;
        this.turretServoA.setDirection(DcMotorSimple.Direction.REVERSE);
        this.turretServoB.setDirection(DcMotorSimple.Direction.REVERSE);
        this.turretEncoder = turretEncoder;

    }
    public void setPower (double power) {
        power = Math.min(power,1);
        power = Math.max(power,-1); //todo write this more readable
        turretServoA.setPower(power);
        turretServoB.setPower(power);
    }
    public double getAngle(){
        return turretEncoder.getCurrentPosition()/TICKS_PER_REV * 360 * GEAR_RATIO;
    }

    public void stop(){
        setPower(0);
    }

    public PIDController getPidController() {
        return pidController;
    }

}

