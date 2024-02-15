package org.firstinspires.ftc.teamcode.Commands.armCommands.elevator;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class ElevatorStayInPlace extends ElevatorGetToHeightPID {
    public ElevatorStayInPlace(Elevator elevator) {
        super(elevator, elevator.getHeight());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
