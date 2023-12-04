package org.firstinspires.ftc.teamcode.Commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class ElevatorStayInPlace extends ElevatorGetToHeightPID {
    public ElevatorStayInPlace(Elevator elevator) {
        super(elevator.getHeight(), elevator);
    }
}
