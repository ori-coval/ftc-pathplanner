package org.firstinspires.ftc.teamcode.Commands.armCommands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;

public class ElevatorUpJoystick extends CommandBase {

    RobotControl robot;

    public ElevatorUpJoystick(RobotControl robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        new ElbowGetToPosition(robot.elbow, 0.4).schedule();
        robot.elevator.setPower(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        robot.elevator.setPower(0);
    }
}
