package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Cartridge extends SubsystemBase {
    public enum State{
        CLOSED(0.3), SEMI_OPEN(0.24), OPEN(0.0);
        final double position;
        State(double position){
            this.position = position;
        }
    }

    private final Servo latch;
    private State lastState = State.CLOSED;

    private final GamepadEx gamepadEx1;
    private boolean isFirstPressRight;
    private boolean isFirstPressLeft;

    private boolean isCartridgeOpenRight;
    private boolean isCartridgeOpenLeft;


    public Cartridge(HardwareMap hardwareMap, GamepadEx gamepadEx1){
        latch = hardwareMap.servo.get("cartridge");
        this.gamepadEx1 = gamepadEx1;
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

        changingStateUsingTrigger(leftTriggerPressed, true);
        changingStateUsingTrigger(rightTriggerPressed, false);

    }

    private void changingStateUsingTrigger(boolean isTriggerPressed, boolean isLeft) { //don't let my algorithms teacher see this. (probably worse code I have ever written)
        if(isLeft) {
            if(isTriggerPressed) {
                if(isFirstPressLeft) {
                    if(isCartridgeOpenLeft) {
                        setState(State.CLOSED);
                        isCartridgeOpenLeft = false;
                    } else {
                        setState(State.SEMI_OPEN);
                        isCartridgeOpenLeft = true;
                    }
                }
                isFirstPressLeft = false;
            } else isFirstPressLeft = true;
        } else {
            if(isTriggerPressed) {
                if(isFirstPressRight) {
                    if(isCartridgeOpenRight) {
                        setState(State.CLOSED);
                        isCartridgeOpenRight = false;
                    } else {
                        setState(State.OPEN);
                        isCartridgeOpenRight = true;
                    }
                }
                isFirstPressRight = false;
            } else isFirstPressRight = true;
        }
    }

    @Override
    public void periodic() {
//        updateLatchPosition();
    }
}