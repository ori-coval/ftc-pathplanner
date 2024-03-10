package org.firstinspires.ftc.teamcode.Commands.auto;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class ScoringSecondPixelAuto extends ScoringCommand {
    public ScoringSecondPixelAuto(RobotControl robot) {
        super(
                new ArmGetToPosition(robot, ArmPosition.SCORE_MID_FAR,robot.allianceColor == AllianceColor.RED),
                new ArmGetToPosition(robot, ArmPosition.SCORE_MID_FAR_2, robot.allianceColor == AllianceColor.RED),
                robot,
                2.
        );
    }
}
