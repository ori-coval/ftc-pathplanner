package org.firstinspires.ftc.teamcode.Commands.armCommands.elevator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

import java.util.Calendar;

public class ElevatorGetToHeightPID extends CommandBase {
    private final Elevator elevator;
    private final double goalHeight;
    private final PIDController pidController;
    private long startTime;
    private long startTime0;
    private boolean isListeningToPID = true;
    private boolean switchWasPressed = false;
    private final long TIME_WAITING_FOR_ELEVATOR_PID = 500; //todo need to tune this
    private final long TIME_WAITING_FOR_ELEVATOR_TO_COME_DOWN = 500; //todo need to tune this
    private final double RESTING_POWER = -0.8; //todo need to tune this


    public ElevatorGetToHeightPID(Elevator elevator, double goalHeight) {
        this.elevator = elevator;
        this.goalHeight = goalHeight;
        pidController = elevator.getPidController();
        pidController.setTolerance(0.5);
        addRequirements(elevator);
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
                elevator.setPower(RESTING_POWER);
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
        elevator.setPower(pidController.calculate(elevator.getHeight()) + elevator.getKg() + Math.signum(pidController.getPositionError()) * elevator.getKs());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPower(0);
        if(switchWasPressed) elevator.resetEncoder();
        FtcDashboard.getInstance().getTelemetry().addLine(String.valueOf(elevator.getHeight()));
        FtcDashboard.getInstance().getTelemetry().update();

    }

    @Override
    public boolean isFinished() {
        if (goalHeight <= 0) {
            if (elevator.getSwitchState()) {
                switchWasPressed = true;
                return true;
            } else return Calendar.getInstance().getTimeInMillis() - startTime0 > TIME_WAITING_FOR_ELEVATOR_TO_COME_DOWN;
        } else {
            return pidController.atSetPoint();
        }
    }
}
