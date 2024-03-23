package org.firstinspires.ftc.teamcode.Commands.auto;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;

public class ScoringSecondCycleAuto extends ScoringCommand {
    public ScoringSecondCycleAuto(RobotControl robot) {
        super(
                new ArmGetToPosition(robot, ArmPosition.SCORE_MID_FRONT, true),
                new ArmGetToPosition(robot, ArmPosition.SCORE_MID_FRONT, true),
                robot,
                2
        );
    }
}
