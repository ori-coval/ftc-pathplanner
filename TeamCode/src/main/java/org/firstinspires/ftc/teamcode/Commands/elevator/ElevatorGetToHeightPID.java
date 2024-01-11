package org.firstinspires.ftc.teamcode.Commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class ElevatorGetToHeightPID extends CommandBase {
    private Elevator elevator;
    private double goalHeight;
    private PIDController pidController;

    public ElevatorGetToHeightPID(double goalHeight, Elevator elevator){
        this.elevator = elevator;
        this.goalHeight = goalHeight;
        pidController = elevator.getPidController();
        addRequirements(elevator);
    }
    @Override
    public void initialize() {
        pidController.setSetPoint(goalHeight);
        pidController.setTolerance(0.5);
    }

    @Override
    public void execute() {
        elevator.setPower(pidController.calculate(elevator.getHeight()) + elevator.getKG());
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
