package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Elevator extends SubsystemBase {
    private DcMotor elevatorMotor;
    private DcMotor encoder;
    private final double LEVELS = 3;
    private final double CIRCUMFERENCE = 0;
    private final double TICKS_PER_REV = 0;
    private final double kg = 0;
    private PIDController pidController = new PIDController(1,0,1);


    public Elevator(DcMotor elevatorMotor) {
        this.elevatorMotor = elevatorMotor;
        this.encoder = encoder;
    }

    public void setPower(double power) {
        elevatorMotor.setPower(power);
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

