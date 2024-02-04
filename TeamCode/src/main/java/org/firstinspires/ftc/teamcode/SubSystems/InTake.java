package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class InTake {
    private DcMotorEx inTakeMotor;
    private Servo inTakeAngle;
    private DigitalChannel limitSwitch;
    public InTake.Roller roller;
    public InTake.Lifter lifter;
    public InTake(HardwareMap hardwareMap){
        inTakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        inTakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        inTakeAngle = hardwareMap.servo.get("intakeServo");
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
            inTakeAngle.setPosition(position);
        }

        public void updatePosition() {
            setPosition(getStackPositionValue());
        }
        @Override
        public void periodic() {
            updatePosition();
        }

    }

    public class Roller extends SubsystemBase {
        private int pixelCount;
        public boolean isOnRelease = false;
        public final double COLLECT_POWER = 1;
        public final double EJECT_POWER = -0.9;
        public int getPixelCount() {
            return pixelCount;
        }
        public void setPower(double power) {
            inTakeMotor.setPower(power);
        }
        public boolean currentSwitchState() {
            return limitSwitch.getState();
        }

        //On Press
        private void updatePixelCount() {
            if (!currentSwitchState()) {
                if (isOnRelease){
                    pixelCount++;
                    isOnRelease = false;
                }
            } else {
                isOnRelease = true;
            }
        }

        public boolean isRobotFull() {
            return getPixelCount() >= 2;
        }
        @Override
        public void periodic() {
            updatePixelCount();
        }

        public void stop() {
            setPower(0);
        }
    }
}
