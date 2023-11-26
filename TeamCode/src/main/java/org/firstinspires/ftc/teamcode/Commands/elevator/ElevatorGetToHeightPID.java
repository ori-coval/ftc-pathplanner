package org.firstinspires.ftc.teamcode.Commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class ElevatorGetToHeightPID extends CommandBase {
    Elevator elevator;
    double goalHeight;
    PIDController pidcontroller = new PIDController(1,0,1);
    public ElevatorGetToHeightPID(double goalHeight, Elevator elevator){
        this.elevator = elevator;
        this.goalHeight = goalHeight;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        pidcontroller.setSetPoint(goalHeight);
        pidcontroller.setTolerance(0.5);
    }

    @Override
    public void execute() {
        elevator.setPower(pidcontroller.calculate(elevator.getHeight()));
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return pidcontroller.atSetPoint();
    }
}
