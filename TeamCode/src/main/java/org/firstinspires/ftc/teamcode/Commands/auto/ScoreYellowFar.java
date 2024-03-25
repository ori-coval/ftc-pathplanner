package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class ScoreYellowFar extends ScoringCommand {
    public ScoreYellowFar(RobotControl robot) {
        super(
                getScoringCommand(robot),
                getSecondScoringCommand(robot),
                robot,
                0
        );
    }

    private static Command getScoringCommand(RobotControl robot) {
        return new ConditionalCommand(
                new DetectionSideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE_RED, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_MID_RED, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_FRONT, true),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new DetectionSideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE_BLUE, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_MID_BLUE, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_FRONT, true),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private static Command getSecondScoringCommand(RobotControl robot) {
        return new ConditionalCommand(
                new DetectionSideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE_2_RED, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_MID_2_RED, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_FAR_2_RED, robot.allianceColor == AllianceColor.RED),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new DetectionSideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE_2_BLUE, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_MID_2_BLUE, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_FAR_2_BLUE, robot.allianceColor == AllianceColor.RED),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

}
