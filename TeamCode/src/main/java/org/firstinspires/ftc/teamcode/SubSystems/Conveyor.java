package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Conveyor extends SubsystemBase {
    private DcMotor motor;
    private DigitalChannel limitSwitch;
    public final double IN_POWER = 0.5;
    public final double OUT_POWER = -0.5;

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
    public void stop(){setPower(0);}
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