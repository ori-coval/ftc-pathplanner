package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Elbow extends SubsystemBase {
    private Servo servoRight;
    private Servo servoLeft;
    private PIDController pidController = new PIDController(0,0,0);
    public Elbow (Servo servoLeft,Servo servoRight, PIDController pidController){
        this.servoLeft = servoLeft;
        this.servoRight = servoRight;
        this.pidController = pidController;
    }
    public void setPosition(double position){
        servoLeft.setPosition(position);
        servoRight.setPosition(position);
    }
    public double getAngle(){
        return servoRight.getPosition();
    }

    public PIDController getPidController() {
        return pidController;
    }
}
