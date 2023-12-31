package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class Extender extends SubsystemBase {

    public enum Length {
        CLOSED(0), MID_WAY(0.25), OPEN(0.5);
        private final double servoPosition;
        Length(double length){
            this.servoPosition = length;
        }
    }
    private Servo linearServo;
    private Length currentLength;

    public Extender (Servo linearServo){
        this.linearServo = linearServo;
    }
    public void setPosition(Length length){
        linearServo.setPosition(length.servoPosition);
        currentLength = length;
    }
    public Length getPosition(){
        return currentLength;
    }
}
