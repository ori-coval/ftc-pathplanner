package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class InTake extends SubsystemBase {
    private DcMotor inTakeMotor;
    private Servo inTakeAngle;
    private GamepadEx gamepadEx;
    public final double COLLECT_POWER = 0.8;
    public final double EJECT_POWER = -0.9;
    public final double[] STACK_POSITION = {0.49, 0.52, 0.55, 0.58, 0.62};
    private double currentStackPosition = STACK_POSITION[4];


    // the first value to 5 pixel is 0.62
    // the second value to 4 pixel is 0.58
    // the third value to 3 pixel is 0.55
    // the fourth value to 2 pixel is 0.52
    // the fifth value to 1 pixel is 0.49
    // כשהאיסוף למעלה 0.67

    public InTake(DcMotor inTakeMotor, Servo inTakeAngle, GamepadEx gamepadEx){
        this.inTakeMotor = inTakeMotor;
        this.inTakeAngle = inTakeAngle;
        this.gamepadEx = gamepadEx;
    }
    public void setPower(double power){
        inTakeMotor.setPower(power);
    }

    public void stop(){
        setPower(0);
    }

    public void setPosition(double position){
        inTakeAngle.setPosition(position);
    }
    public double getPosition(){return inTakeAngle.getPosition();}

    public void setStackPosition(int position) {
        currentStackPosition = STACK_POSITION[position];
    }

    public double getStackPosition() {
        return currentStackPosition;
    }

    public void updatePosition() {
        /*I've made it this way in order to have a way to access the stack position in the future.*/
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_UP)) setStackPosition(4);
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT)) setStackPosition(3);
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN)) setStackPosition(2);
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)) setStackPosition(1);
        /*I've made the back button (temporarily) as the default configuration of the intake (the lowest position it needs to be in)*/
        if(gamepadEx.getButton(GamepadKeys.Button.BACK)) setPosition(0);
        setPosition(getStackPosition());
    }

    @Override
    public void periodic() {
        updatePosition();
    }
}
