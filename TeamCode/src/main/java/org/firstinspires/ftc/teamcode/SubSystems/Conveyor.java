package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Conveyor extends SubsystemBase {
    private DigitalChannel limitSwitch;
    private int pixelCount = 0;
    private boolean wasPressed;

    public Conveyor(int pixelCount, DigitalChannel limitSwitch){
        this.pixelCount = pixelCount;
        this.limitSwitch = limitSwitch;
    }
    public boolean isPressed(){
        return limitSwitch.getState();
    }
    private void updatePixelCount(){
        if (wasPressed && !isPressed()){pixelCount += 1;}
        if (isPressed()){wasPressed = true;}
    }
    public int getPixelCount(){return pixelCount;}

    public boolean isRobotFull(){
        return getPixelCount() >= 2;
    }
    @Override
    public void periodic() {
        updatePixelCount();
    }
}