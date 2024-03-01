package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

import java.util.Calendar;

public class Elbow extends SubsystemBase {
    private final Servo servoRight;
    private final Servo servoLeft;
    private final DigitalChannel sensor;
    private final RobotControl robot;
    public boolean updateSafePlace = false;
    private boolean inSafePlace = false;
    private boolean lastState = false;
    public Elbow (HardwareMap hardwareMap, RobotControl robot) {
        this.robot = robot;
        servoLeft = hardwareMap.servo.get(Configuration.ELBOW_LEFT);
        servoRight = hardwareMap.servo.get(Configuration.ELBOW_RIGHT);
        sensor = hardwareMap.digitalChannel.get(Configuration.SAFE_PLACE_SWITCH);
        servoLeft.setDirection(Servo.Direction.REVERSE); //reverse = 1 - pos
        servoRight.setDirection(Servo.Direction.REVERSE);
    }
    public void setPosition(double position) {
        position = Math.max(position, 0.05);
        servoLeft.setPosition(position - 0.05);
        servoRight.setPosition(position);
    }

    public double getServoPosition(){
        return servoRight.getPosition();
    }
    public boolean getSwitchState() {
        return !sensor.getState();
    }
    public void updateSafeState() {
        if (getSwitchState() && !lastState) {
            if(robot.elevator.getHeight() < 1) {
                inSafePlace = false;
            } else {
                inSafePlace = !inSafePlace;
            }
            updateSafePlace = false;
        }
        lastState = getSwitchState();
    }

    public boolean isInSafePlace() {
        return inSafePlace;
    }

    @Override
    public void periodic() {
        if(updateSafePlace) {
            updateSafeState();
            FtcDashboard.getInstance().getTelemetry().addData("updateSafePlace", Calendar.getInstance().getTimeInMillis());
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}
