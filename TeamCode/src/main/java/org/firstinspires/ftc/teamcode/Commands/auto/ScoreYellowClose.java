package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class ScoreYellowClose extends SequentialCommandGroup {
    public ScoreYellowClose(RobotControl robot) {
        addCommands(
                new ParallelCommandGroup(
                        new ArmGetToPosition(robot, ArmPosition.SCORING, isRobotLeft(robot)),
                        scoreYellowTrajectory(robot)
                ),
                new DetectionSideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE_RED, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_MID_RED, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_FAR_RED, robot.allianceColor == AllianceColor.RED),
                        () -> robot.teamPropDetector.getTeamPropSide()
                )
        );
    }

    private boolean isRobotLeft(RobotControl robot) {
        return (robot.allianceColor == AllianceColor.RED) == (robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE);
    }

    private Command scoreYellowTrajectory(RobotControl robot) {
        return new DetectionSideCommandSwitch(
                new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow (Far Detected)"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow (Center Detected)"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow (Close Detected) Red"), robot.autoDriveTrain),
                () -> robot.teamPropDetector.getTeamPropSide()
        );
    }
}
