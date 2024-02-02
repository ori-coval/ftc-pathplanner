package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Calendar;

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

    private GamepadEx gamepadEx1;
    private long time;

    public Cartridge(HardwareMap hardwareMap, GamepadEx gamepadEx1){
        latch = hardwareMap.servo.get("cartridge");
        this.gamepadEx1 = gamepadEx1;
        time = Calendar.getInstance().getTimeInMillis();
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

    public void updateLatchPosition() {
        boolean leftTriggerPressed = gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;
        boolean rightTriggerPressed = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;

        if(leftTriggerPressed && !rightTriggerPressed) {
            setState(State.SEMI_OPEN);
        }
        if(rightTriggerPressed) {
            setState(State.OPEN);
            time = Calendar.getInstance().getTimeInMillis();
        } else if(Calendar.getInstance().getTimeInMillis() - time > 2000) {
            setState(State.CLOSED);
        }

    }

    @Override
    public void periodic() {
        updateLatchPosition();
    }
}
