package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class InTake extends SubsystemBase {
    private DcMotor inTakeMotor;
    private Servo inTakeAngle;
    private Gamepad gamepad;
    public final double COLLECT_POWER = 1;
    public final double EJECT_POWER = -0.9;
    public final double[] STACK_POSITION = {0, 0.07, 0.13, 0.21, 0.77};
    /*
    0.77 - The default position (Highest)
    0.21 - Before the 5th pixel
    0.13 - The first pixel
    0.07 - Next pixels
    0 - lowest position
    */
    private double currentStackPosition = STACK_POSITION[4];

    public InTake(DcMotor inTakeMotor, Servo inTakeAngle, Gamepad gamepad){
        this.inTakeMotor = inTakeMotor;
        this.inTakeAngle = inTakeAngle;
        this.gamepad = gamepad;
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
        setPosition(getStackPosition());
        /*
        I think that using 5 buttons for the intake is incredibly wasteful,
        in my opinion it'll be better using some kind of steeper mechanism that whenever I press some kind of button
        it goes between states of the intake. Like the more I press the higher it gets. That way we can use only one button.
         */
    }

    @Override
    public void periodic() {
        updatePosition();
    }
}
