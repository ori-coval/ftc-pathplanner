package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Elevator extends SubsystemBase {
    private DcMotor[] elevatorMotors = new DcMotor[3];
    private DcMotor encoder;
    private final double LEVELS = 3;
    private final double CIRCUMFERENCE = 0;
    private final double TICKS_PER_REV = 0;
    private final double kg = 0;
    private PIDController pidController = new PIDController(1,0,1);


    public Elevator(DcMotor elevatorMotor1, DcMotor elevatorMotor2, DcMotor elevatorMotor3) {
        this.elevatorMotors[0] = elevatorMotor1;
        this.elevatorMotors[1] = elevatorMotor2;
        this.elevatorMotors[2] = elevatorMotor3;
        elevatorMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.encoder = elevatorMotor1;
    }

    public void setPower(double power) {
        for (DcMotor motor: elevatorMotors) {
            motor.setPower(power);
        }
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

