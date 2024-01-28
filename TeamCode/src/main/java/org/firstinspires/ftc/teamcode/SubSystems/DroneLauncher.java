package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher extends SubsystemBase {
    public enum State{
        HOLD(0.0), RELEASE(0.0);
        final double position;
        State(double position){
            this.position = position;
        }
    }

    private Servo drone;
    private State lastState = State.HOLD;

    public DroneLauncher(HardwareMap hardwareMap){drone = hardwareMap.servo.get("drone");
    }


    public void setState(State state){
        drone.setPosition(state.position);
        lastState = state;
    }

    public void holdDrone() {
        setState(State.HOLD);
    }
    public void releaseDrone() {
        setState(State.RELEASE);
    }

    public State getState(){
        return lastState;
    }


}
