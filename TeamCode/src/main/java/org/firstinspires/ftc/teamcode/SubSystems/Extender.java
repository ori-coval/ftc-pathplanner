package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class Extender extends SubsystemBase {
    private Servo linearServo;
    public Extender (Servo linearServo){
        this.linearServo = linearServo;
    }
    public void setPosition(double position){
        linearServo.setPosition(position);

    }
    public double getPosition(){
        return linearServo.getPosition();
    }
}
