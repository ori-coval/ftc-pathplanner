package org.firstinspires.ftc.teamcode.Commands.armCommands.elevator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotControl;

import java.util.Calendar;

public class ElevatorGetToHeightPID extends CommandBase {
    private final RobotControl robot;
    private final double goalHeight;
    private final PIDController pidController;
    private long startTime0;
    private boolean isListeningToPID = true;
    private boolean switchWasPressed = false;
    private final long TIME_WAITING_FOR_ELEVATOR_TO_COME_DOWN = 100; //todo need to tune this
    private final double RESTING_POWER = -0.8; //todo need to tune this


    public ElevatorGetToHeightPID(RobotControl robot, double goalHeight) {
        this.robot = robot;
        this.goalHeight = goalHeight;
        pidController = robot.elevator.getPidController();
        pidController.setTolerance(0.5);
        addRequirements(robot.elevator);
    }

    @Override
    public void initialize() {
        pidController.setSetPoint(goalHeight);
        isListeningToPID = true;
        switchWasPressed = false;
    }

    @Override
    public void execute() {
        if (goalHeight <= 0) {
            if (!isListeningToPID || pidController.atSetPoint()) {
                isListeningToPID = false;
                robot.elevator.setPower(RESTING_POWER);
            } else {
                activatePID();
                startTime0 = Calendar.getInstance().getTimeInMillis();
            }
        } else {
            activatePID();
        }
        FtcDashboard.getInstance().getTelemetry().addData("elevator is finished", isFinished());

    }

    private void activatePID() {
        robot.elevator.setPower(pidController.calculate(robot.elevator.getHeight()) + robot.elevator.getKg() + Math.signum(pidController.getPositionError()) * robot.elevator.getKs());
    }

    @Override
    public void end(boolean interrupted) {
        robot.elevator.setPower(0);
        if(switchWasPressed) robot.elevator.resetEncoder();
        FtcDashboard.getInstance().getTelemetry().addLine(String.valueOf(robot.elevator.getHeight()));
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public boolean isFinished() {
        if (goalHeight <= 0) {
            if (robot.elevator.getSwitchState()) {
                switchWasPressed = true;
                return true;
            } else return Calendar.getInstance().getTimeInMillis() - startTime0 > TIME_WAITING_FOR_ELEVATOR_TO_COME_DOWN;
        } else {
            return pidController.atSetPoint();
        }
    }
}
