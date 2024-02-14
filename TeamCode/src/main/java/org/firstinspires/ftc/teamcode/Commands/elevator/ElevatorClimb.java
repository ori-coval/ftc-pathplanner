package org.firstinspires.ftc.teamcode.Commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class ElevatorClimb extends ElevatorGetToHeightPID {

    public static final int LOW_CLIMB_HEIGHT = 20;
    Elevator elevator;

    public ElevatorClimb(Elevator elevator) {
        super(elevator, LOW_CLIMB_HEIGHT);
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        super.execute();
        // activate the pully
        elevator.climberSetPower(-1);
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        elevator.climberSetPower(0);
    }
}
