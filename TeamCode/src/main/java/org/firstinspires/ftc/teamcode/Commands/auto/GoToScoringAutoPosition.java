package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class GoToScoringAutoPosition extends ConditionalCommand {
    public GoToScoringAutoPosition(RobotControl robot) {
        super(
                new ArmGetToPosition(robot, ArmPosition.SCORING_AUTO, robot.allianceColor == AllianceColor.RED),
                new InstantCommand(),
                () -> robot.teamPropDetector.getTeamPropSide() != DetectionSide.CLOSE
        );
    }
}
