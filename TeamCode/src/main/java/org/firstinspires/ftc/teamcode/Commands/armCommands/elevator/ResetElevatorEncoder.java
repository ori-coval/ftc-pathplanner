package org.firstinspires.ftc.teamcode.Commands.armCommands.elevator;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotControl;

public class ResetElevatorEncoder extends ConditionalCommand {
    public ResetElevatorEncoder(RobotControl robot) {
        super(
                new InstantCommand(() -> robot.elevator.resetEncoder()),
                new InstantCommand(),
                () -> robot.inDebugMode
        );
    }
}
