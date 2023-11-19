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
        pidcontroller.setSetPoint(1);
    }
    @Override
    public void execute() {
        elevator.setPower(pidcontroller.calculate(goalHeight));
    }

    @Override
    public void end(boolean interrupted) {
        elevator.reset();
    }

    @Override
    public boolean isFinished() {
        return pidcontroller.atSetPoint();
    }
}
