package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Extender extends SubsystemBase {
    private Servo linearServo;
    private final double elbow0 = 1;
    private final double elbow1 = 1;
    private double lastLength;
    private double lengthToRotation(double length) {
        lastLength = length;
        //cosine law
        double lengthInRadians = Math.acos((elbow1*elbow1 + length*length - elbow0*elbow0) / (2 * elbow1 * length));
        return lengthInRadians/(2*Math.PI);
    }
    public Extender (Servo linearServo){
        this.linearServo = linearServo;
    }

    public void setPosition(double pos) {
        linearServo.setPosition(pos);
    }

    public void setLength(double length){
        lastLength = length;
//        linearServo.setPosition(lengthToRotation(length));
        linearServo.setPosition(length);
    }
    public double getLength(){
        return lastLength;
    }

}
