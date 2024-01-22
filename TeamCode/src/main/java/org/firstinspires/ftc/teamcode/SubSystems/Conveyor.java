package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Conveyor extends SubsystemBase {
    private DigitalChannel limitSwitch;
    private int pixelCount;
    private boolean lastState = false;

    public Conveyor(int pixelCount, DigitalChannel limitSwitch){
        this.pixelCount = pixelCount;
        this.limitSwitch = limitSwitch;
    }
    public boolean currentState(){
        return limitSwitch.getState();
    }
    private void updatePixelCount(){
        if (!lastState && currentState()){
            pixelCount++;
        }
        lastState = currentState();

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