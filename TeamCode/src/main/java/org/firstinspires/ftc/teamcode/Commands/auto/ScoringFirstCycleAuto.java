package org.firstinspires.ftc.teamcode.Commands.auto;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class ScoringFirstCycleAuto extends ScoringCommand {
    public ScoringFirstCycleAuto(RobotControl robot) {
        super(
                new ArmGetToPosition(robot, ArmPosition.SCORE_MID_FRONT,robot.allianceColor == AllianceColor.RED),
                new ArmGetToPosition(robot, ArmPosition.SCORE_MID_FRONT, robot.allianceColor == AllianceColor.RED),
                robot,
                1
        );
    }
}
