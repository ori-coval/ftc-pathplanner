package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Elevator extends SubsystemBase {
    private DcMotor motorUp;
    private DcMotor motorMid;
    private DcMotor motorDown;
    private DcMotor encoder;
    private final double LEVELS = 3;
    private final double CIRCUMFERENCE = 0;
    private final double TICKS_PER_REV = 0;
    private final double kg = 0;
    public final double TOP_DIST_FROM_FLOOR = 0;
    private PIDController pidController = new PIDController(1,0,1);


    public Elevator(DcMotor motorDown,DcMotor motorMid,DcMotor motorUp) {
        this.motorUp = motorUp;
        this.motorMid = motorMid;
        this.motorDown = motorDown;
        this.encoder = motorDown;
        this.motorMid.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        motorDown.setPower(power);
        motorMid.setPower (power);
        motorUp.setPower  (power);
    }
    public double getEncoderValue(){
        return encoder.getCurrentPosition();
    }
    public double getHeight(){
        double PULLED_LENGTH = CIRCUMFERENCE * encoder.getCurrentPosition() / TICKS_PER_REV;
        return LEVELS * PULLED_LENGTH;
    }

    public double getKg() {
        return kg;
    }

    public PIDController getPidController() {
        return pidController;
    }
}

