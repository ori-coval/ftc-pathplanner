package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Elbow extends SubsystemBase {
    private Servo servoRight;
    private Servo servoLeft;
    private DigitalChannel sensor;
    private boolean inSafePlace = false;
    private boolean lastState = false;
    public Elbow (HardwareMap hardwareMap){
        servoLeft = hardwareMap.servo.get(Configuration.ELBOW_LEFT);
        servoRight = hardwareMap.servo.get(Configuration.ELBOW_RIGHT);
        servoLeft.setDirection(Servo.Direction.REVERSE); //reverse = 1 - pos
        servoRight.setDirection(Servo.Direction.REVERSE);
    }
    public void setPosition(double position) {
        position = Math.max(position, 0.05);
        servoLeft.setPosition(position - 0.05);
        servoRight.setPosition(position);
    }

    public double getServoPosition(){
        return servoRight.getPosition();
    }
    public void updateSwitch(){
        if (sensor.getState() & !lastState){
            inSafePlace = !inSafePlace;
        }
    }

    @Override
    public void periodic() {
        updateSwitch();
        lastState = sensor.getState();
    }
}
