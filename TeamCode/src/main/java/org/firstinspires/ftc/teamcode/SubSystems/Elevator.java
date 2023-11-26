package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Elevator extends SubsystemBase {

    //how do we get the height? do we just get a number or is some sort of conversion needed
    //like with angles and ticks?
    //also we need to decide how to control the elevator (servos, encoders, etc.)
    private DcMotor elevatorMotor;
    DcMotor encoder;
    private final double LEVELS = 3;
    private final double CIRCUMFERENCE = 0;
    private final double TICKS_PER_REV = 0;

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


}

