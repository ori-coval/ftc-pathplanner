package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Elbow extends SubsystemBase {
    private Servo servoRight;
    private Servo servoLeft;
    private AnalogInput encoder;
    private final double OFFSET = 0.722;
    private final double ENCODER_RATIO = 0.5/0.228;
    private PIDController pidController = new PIDController(0,0,0);
    public Elbow (HardwareMap hardwareMap){
        servoLeft = hardwareMap.servo.get("elbowLeft");
        servoRight = hardwareMap.servo.get("elbowRight");
        encoder = hardwareMap.analogInput.get("elbowEncoder");
        servoLeft.setDirection(Servo.Direction.REVERSE);
    }
    public void setPosition(double position){
        servoLeft.setPosition(position);
        servoRight.setPosition(position);
    }
    public double getEncoderPosition() {
        return (-1) * (encoder.getVoltage() / encoder.getMaxVoltage() - OFFSET) * ENCODER_RATIO;
        //The encoder gives negative values
    }

    public double getServoPosition(){
        return servoLeft.getPosition();
    }



    public PIDController getPidController() {
        return pidController;
    }
}
