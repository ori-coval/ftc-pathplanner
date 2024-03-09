package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Cartridge extends SubsystemBase {
    public enum State {
        CLOSED_ONE_PIXEL(0.9), CLOSED_TWO_PIXELS(0.78), SEMI_OPEN(0.515), OPEN(0.55), INTAKE_OPEN(0.55 /*This needs to be 0.132*/); //TODO: NEW CARTRIDGE POS
        final double position;
        State(double position){
            this.position = position;
        }
    }

    private final Servo latch;
    private State lastState = State.CLOSED_TWO_PIXELS;


    public  Cartridge(HardwareMap hardwareMap) {
        latch = hardwareMap.servo.get(Configuration.CARTRIDGE);
    }

    public void setPosition(double position){
        latch.setPosition(position);
    }
    public double getPosition(){
        return latch.getPosition();
    }
    public void setState(State state){
        latch.setPosition(state.position);
        lastState = state;
    }

    public State getState(){
        return lastState;
    }
}