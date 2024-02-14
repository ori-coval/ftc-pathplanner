package org.firstinspires.ftc.teamcode.Commands.elevator;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class ElevatorClimb extends ElevatorGetToHeightPID {

    public static final int LOW_CLIMB_HEIGHT = 10;
    public static final int TICK_TOLERANCE_PER_CYCLE = 3;
    Elevator elevator;
    double lastPosition;

    public ElevatorClimb(Elevator elevator) {
        super(elevator, LOW_CLIMB_HEIGHT);
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        super.execute();
        lastPosition = elevator.getClimberPosition();
        // activate the pully
        elevator.climberSetPower(-1);
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        elevator.climberSetPower(0);
    }

    @Override
    public boolean isFinished() {
        boolean isStuck = Math.abs(lastPosition-elevator.getClimberPosition()) > TICK_TOLERANCE_PER_CYCLE;
        return super.isFinished() || isStuck;

    }
}
