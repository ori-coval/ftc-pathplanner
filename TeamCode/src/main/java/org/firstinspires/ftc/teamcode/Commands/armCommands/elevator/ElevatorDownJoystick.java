package org.firstinspires.ftc.teamcode.Commands.armCommands.elevator;

import org.firstinspires.ftc.teamcode.RobotControl;

public class ElevatorDownJoystick extends ElevatorJoystick {

    private static final double ELEVATOR_POWER = -1;

    public ElevatorDownJoystick(RobotControl robot) {
        super(
                ELEVATOR_POWER,
                robot
        );
    }


}
