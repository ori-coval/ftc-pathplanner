package org.firstinspires.ftc.teamcode.Commands.drivetrain;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotControl;

public class ResetFieldOriented extends InstantCommand {
    public ResetFieldOriented(RobotControl robot) {
        new InstantCommand(() -> robot.driveTrain.setHeadingOffset(robot.driveTrain.getPoseEstimate().getHeading()));
    }
}
