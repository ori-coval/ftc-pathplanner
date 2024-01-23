package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

public class Elbow extends SubsystemBase {
    private Servo servoRight;
    private Servo servoLeft;
    private AnalogInput encoder;
    private final double OFFSET = 0.183 + 0.089 + 0.45;
    private PIDController pidController = new PIDController(0,0,0);
    public Elbow (Servo servoLeft, Servo servoRight, AnalogInput encoder){
        this.servoLeft = servoLeft;
        this.servoRight = servoRight;
        this.encoder = encoder;
        this.servoLeft.setDirection(Servo.Direction.REVERSE);
    }
    public void setPosition(double position){
        servoLeft.setPosition (position);
        servoRight.setPosition(position);
    }
    public double getPosition() {
        return (-1) * (encoder.getVoltage() / encoder.getMaxVoltage() - OFFSET) * (0.5/0.228);
    }

    public PIDController getPidController() {
        return pidController;
    }
}
