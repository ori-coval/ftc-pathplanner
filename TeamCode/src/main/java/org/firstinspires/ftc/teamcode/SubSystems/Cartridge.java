package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class Cartridge extends SubsystemBase {
    public enum State{
        CLOSED(0), SEMI_OPEN(0.125), OPEN(0.25);
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


    public void setState(State state){
        latch.setPosition(state.position);
        lastState = state;
    }

    public void openCartidge() {
        setState(State.OPEN);
    }
    public void closedCartidge() {
        setState(State.CLOSED);
    }
    public void semiOpenSartidge() {
        setState(State.SEMI_OPEN);
    }
    public State getState(){
        return lastState;
    }


}
