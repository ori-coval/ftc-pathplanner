package org.firstinspires.ftc.teamcode.Commands.armCommands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotControl;

public class ElevatorGoUp extends CommandBase {

    RobotControl robot;

    public ElevatorGoUp(RobotControl robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.elevator.setPower(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        robot.elevator.setPower(0);
    }
}
