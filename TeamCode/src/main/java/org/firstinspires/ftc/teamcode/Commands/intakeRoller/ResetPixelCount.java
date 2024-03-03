package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotControl;

public class ResetPixelCount extends InstantCommand {
    public ResetPixelCount(RobotControl robot) {
        super(() -> robot.intake.roller.setPixelCount(0));
    }
}
