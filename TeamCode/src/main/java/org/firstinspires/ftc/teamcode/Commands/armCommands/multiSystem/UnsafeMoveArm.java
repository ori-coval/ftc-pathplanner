package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.RobotControl;

public class UnsafeMoveArm extends ConditionalCommand {

    public UnsafeMoveArm(RobotControl robot, ArmPosition position, boolean isLeftOfBoard) {
        super(
                new UnsafeMoveArmUp(robot, position, isLeftOfBoard),
                new UnsafeMoveArmDown(robot, position, isLeftOfBoard),
                () -> (ArmGetToPosition.lastPosition.getElevatorHeight() < position.getElevatorHeight())
        );
    }
}