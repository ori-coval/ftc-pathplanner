package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Conveyor extends SubsystemBase {
    private DcMotor motor;
    private DigitalChannel limitSwitch;
    private int pixelCount = 0;
    private boolean wasPressed;

    public Conveyor(DcMotor motor, DigitalChannel limitSwitch, int pixelCount){
        this.limitSwitch = limitSwitch;
        this.motor = motor;
        this.pixelCount = pixelCount;
    }
    public void setPower(double power){
        motor.setPower(power);
    }
    public boolean isPressed(){
        return limitSwitch.getState();
    }
    private void updatePixelCount(){
        if (wasPressed && !isPressed()){pixelCount += 1;}
        if (isPressed()){wasPressed = true;}
    }
    public int getPixelCount(){return pixelCount;}

    @Override
    public void periodic() {
        updatePixelCount();
    }
}