package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotControl;

public class InstantIntakeRotate extends InstantCommand {
    public InstantIntakeRotate(RobotControl robot, double power) {
        super(() -> robot.intake.roller.setPower(power));
    }
}
