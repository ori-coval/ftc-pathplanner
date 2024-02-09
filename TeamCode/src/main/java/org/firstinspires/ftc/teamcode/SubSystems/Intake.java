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
        public final double[] STACK_POSITION = {0, 0.10, 0.13, 0.21, 0.45};
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
            intakeMotor.setPower(power);
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
