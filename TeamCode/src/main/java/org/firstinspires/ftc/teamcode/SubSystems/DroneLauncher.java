package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class DroneLauncher extends SubsystemBase {
    public enum State {
        HOLD(0.4), RELEASE(1);
        final double position;
        State(double position){
            this.position = position;
        }
    }

    private final Servo drone;
    private State lastState = State.HOLD;

    public DroneLauncher(HardwareMap hardwareMap) {
        drone = hardwareMap.servo.get(Configuration.DRONE);
    }

    public void setPosition(double position) {
        drone.setPosition(position);
    }

    public double getPosition() {
        return drone.getPosition();
    }

    public void setState(State state) {
        setPosition(state.position);
        lastState = state;
    }

    public State getState(){
        return lastState;
    }


}
