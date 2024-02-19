package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ArmGetToSelectedPosition extends ConditionalCommand {
    private static final ArmPosition[] armPositions = ArmPosition.values();

        public ArmGetToSelectedPosition(RobotControl robot) {
        super(
                getGroup(true, robot),
                getGroup(false, robot),
                ArmPositionSelector::getIsLeftOfBoard
        );
    }

    private static SequentialCommandGroup getGroup(boolean isLeftOfBoard, RobotControl robot) {
        return new SequentialCommandGroup() {{
            for(ArmPosition armPosition : armPositions) {
                addCommands(new ConditionalCommand(
                        new ArmGetToPosition(robot, armPosition, isLeftOfBoard),
                        new InstantCommand(),
                        () -> armPosition == ArmPositionSelector.getPosition()
                ));
            }
        }};
    }
}