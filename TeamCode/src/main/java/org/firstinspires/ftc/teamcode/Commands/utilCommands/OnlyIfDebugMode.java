package org.firstinspires.ftc.teamcode.Commands.utilCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotControl;

public class OnlyIfDebugMode extends ConditionalCommand {
    public OnlyIfDebugMode(Runnable runnable, RobotControl robot) {
        super(
                new InstantCommand(runnable),
                new InstantCommand(),
                () -> robot.inDebugMode
        );
    }
}
