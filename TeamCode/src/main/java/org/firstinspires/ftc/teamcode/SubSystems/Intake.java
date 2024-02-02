package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx intakeMotor;
    private Servo intakeAngle;
    private DigitalChannel limitSwitch;
    public Intake.Roller roller;
    public Intake.Lifter lifter;
    public Intake(HardwareMap hardwareMap){
        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeAngle = hardwareMap.servo.get("intakeServo");
        limitSwitch = hardwareMap.digitalChannel.get("switch");
        roller = new Roller();
        lifter = new Lifter();
    }
    public class Lifter extends SubsystemBase {
        public final double[] STACK_POSITION = {0, 0.07, 0.13, 0.21, 0.77};
        /*
        0.77 - The default position (Highest)
        0.21 - Before the 5th pixel
        0.13 - The first pixel
        0.07 - Next pixels
        0 - lowest position
        */
        private int currentStackPosition = 4;
        private double currentIntakePosition = STACK_POSITION[currentStackPosition];

        public void setStackPosition(int position) {
            currentStackPosition = position;
            currentIntakePosition = STACK_POSITION[currentStackPosition];
        }
        public int getStackPosition() {
            return currentStackPosition;
        }
        public double getStackPositionValue() {
            return currentIntakePosition;
        }
        public void setPosition(double position){
            intakeAngle.setPosition(position);
        }

        public void updatePosition() {
            setPosition(getStackPositionValue());
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

    public class Roller extends SubsystemBase {
        private int pixelCount;
        private boolean isSwitchReleased = false;
        public boolean lastButtonStateOnPress = false;
        private boolean lastButtonStateOnRelease = false;
        public final double COLLECT_POWER = 1;
        public final double EJECT_POWER = -0.9;
        public void setPower(double power) {
            intakeMotor.setPower(power);
        }
        public void stop() {
            setPower(0);
        }
        public boolean currentSwitchState() {
            return limitSwitch.getState();
        }

        //On Release
        public void updateIsSwitchReleased() {
            if(lastButtonStateOnRelease && !currentSwitchState()) {
                isSwitchReleased = true;
                lastButtonStateOnRelease = false;
            }
            if(currentSwitchState()) lastButtonStateOnRelease = true;
        }

        //On Press
        private void updatePixelCount() {
            if (!lastButtonStateOnPress && currentSwitchState()) { //TODO: Sometimes this condition isn't met when the button is pressed. (I honestly don't know why)
                pixelCount++;
                isSwitchReleased = false;
            }
            lastButtonStateOnPress = currentSwitchState();
        }

        public boolean getIsSwitchReleased() {
            return isSwitchReleased;
        }
        public int getPixelCount() {
            return pixelCount;
        }
        public boolean isRobotFull() {
            return (getPixelCount() >= 2) && getIsSwitchReleased();
        }
        @Override
        public void periodic() {
            updatePixelCount();
            updateIsSwitchReleased();
        }
    }
}
