package org.firstinspires.ftc.teamcode.Commands.elevator;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class ElevatorClimbAndStayInPlace extends SequentialCommandGroup {

    public ElevatorClimbAndStayInPlace(Elevator elevator){
        super(new ElevatorClimb(elevator),new ElevatorStayInPlace(elevator));
    }






}
