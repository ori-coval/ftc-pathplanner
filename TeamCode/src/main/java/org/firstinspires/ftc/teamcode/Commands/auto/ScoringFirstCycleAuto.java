package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class ScoringFirstCycleAuto extends ScoringCommand {
    public ScoringFirstCycleAuto(RobotControl robot) {
        super(
                getScoringCommand(robot),
                getScoringCommand(robot),
                robot,
                1
        );
    }

    private static Command getScoringCommand(RobotControl robot) {
        return new ConditionalCommand(
                new ArmGetToPosition(robot, ArmPosition.SCORE_FIRST_CYCLE_LOW_RED,robot.allianceColor == AllianceColor.RED),
                new ArmGetToPosition(robot, ArmPosition.SCORE_FIRST_CYCLE_LOW_BLUE,robot.allianceColor == AllianceColor.RED),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

}
