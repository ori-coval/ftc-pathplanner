package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class Extender extends SubsystemBase {
    private Servo linearServo;
    public Extender (Servo linearServo){
        this.linearServo = linearServo;
    }
    public void setLength(double length){
        linearServo.setPosition(length);
    }
    public double getLength(){
        return linearServo.getPosition();
    }
}
