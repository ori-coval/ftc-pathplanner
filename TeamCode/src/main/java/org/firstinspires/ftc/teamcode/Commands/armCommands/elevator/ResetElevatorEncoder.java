package org.firstinspires.ftc.teamcode.Commands.armCommands.elevator;

import org.firstinspires.ftc.teamcode.Commands.utilCommands.OnlyIfDebugMode;
import org.firstinspires.ftc.teamcode.RobotControl;

public class ResetElevatorEncoder extends OnlyIfDebugMode {
    public ResetElevatorEncoder(RobotControl robot) {
        super(
                () -> robot.elevator.resetEncoder(),
                robot
        );
    }
}
