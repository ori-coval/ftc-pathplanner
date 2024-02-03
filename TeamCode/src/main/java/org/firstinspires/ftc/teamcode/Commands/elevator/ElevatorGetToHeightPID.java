package org.firstinspires.ftc.teamcode.Commands.elevator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class ElevatorGetToHeightPID extends CommandBase {
    private Elevator elevator;
    private double goalHeight;
    private PIDController pidController;

    public ElevatorGetToHeightPID(Elevator elevator, double goalHeight){
        this.elevator = elevator;
        this.goalHeight = goalHeight;
        pidController = elevator.getPidController();
        pidController.setTolerance(1.5);
        addRequirements(elevator);
    }
    @Override
    public void initialize() {
        pidController.setSetPoint(goalHeight);

    }

    @Override
    public void execute() {
        elevator.setPower(pidController.calculate(elevator.getHeight()) + elevator.getKF());
        FtcDashboard.getInstance().getTelemetry().addData("elevator is finished", isFinished());

    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint();
    }
}
