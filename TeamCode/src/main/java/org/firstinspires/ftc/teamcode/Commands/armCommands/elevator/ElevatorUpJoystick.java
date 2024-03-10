package org.firstinspires.ftc.teamcode.Commands.armCommands.elevator;

import org.firstinspires.ftc.teamcode.RobotControl;

public class ElevatorUpJoystick extends ElevatorJoystick {

    private static final double ELEVATOR_POWER = 0.9;

    public ElevatorUpJoystick(RobotControl robot) {
        super(
                ELEVATOR_POWER,
                robot
        );
    }


}
