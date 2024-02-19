package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Elbow extends SubsystemBase {
    private Servo servoRight;
    private Servo servoLeft;
    public Elbow (HardwareMap hardwareMap){
        servoLeft = hardwareMap.servo.get(Configuration.ELBOW_LEFT);
        servoRight = hardwareMap.servo.get(Configuration.ELBOW_RIGHT);
        servoLeft.setDirection(Servo.Direction.REVERSE); //reverse = 1 - pos
        servoRight.setDirection(Servo.Direction.REVERSE);
    }
    public void setPosition(double position) {
        position = Math.max(position, 0.1);
        servoLeft.setPosition(position - 0.1);
        servoRight.setPosition(position);
    }

    public double getServoPosition(){
        return servoRight.getPosition();
    }

}
