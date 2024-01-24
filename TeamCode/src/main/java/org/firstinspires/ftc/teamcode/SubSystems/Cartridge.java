package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class Cartridge extends SubsystemBase {
    public enum State{
        CLOSED(0.3), SEMI_OPEN(0.24), OPEN(0.0);
        final double position;
        State(double position){
            this.position = position;
        }
    }

    private Servo latch;
    private State lastState = State.CLOSED;

    public Cartridge(Servo latch){
        this.latch = latch;
    }


    public void setPosition(double position){latch.setPosition(position);}
    public double getPosition(){return latch.getPosition();}
    public void setState(State state){
        latch.setPosition(state.position);
        lastState = state;
    }

    public void openCartridge() {
        setState(State.OPEN);
    }
    public void closedCartridge() {
        setState(State.CLOSED);
    }
    public void semiOpenCartridge() {
        setState(State.SEMI_OPEN);
    }
    public State getState(){
        return lastState;
    }


}
