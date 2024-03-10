package org.firstinspires.ftc.teamcode.Commands.armCommands.turret;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotControl;

public class ResetTurretEncoder extends ConditionalCommand {
    public ResetTurretEncoder(RobotControl robot) {
        super(
                new InstantCommand(() -> robot.turret.resetEncoder()),
                new InstantCommand(),
                () -> robot.inDebugMode
        );
    }
}
