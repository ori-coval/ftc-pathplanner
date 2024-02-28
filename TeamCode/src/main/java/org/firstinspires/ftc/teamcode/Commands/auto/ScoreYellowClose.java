package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class ScoreYellowClose extends SequentialCommandGroup {
    public ScoreYellowClose(RobotControl robot) {
        addCommands(
//                new ArmGetToPosition(robot, ArmPosition.SCORING, isRobotLeft(robot)),
                scoreYellowTrajectory(robot),
                new InstantCommand(() -> {
                    robot.telemetry.addData("isRobotLeft", isRobotLeft(robot));
                    robot.telemetry.addLine(String.valueOf(robot.teamPropDetector.getTeamPropSide()));
                    robot.telemetry.update();
                })

        );
    }

    private boolean isRobotLeft(RobotControl robot) {
        return (robot.allianceColor == AllianceColor.RED) == (robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE);
    }

    private Command scoreYellowTrajectory(RobotControl robot) {
        return new SideCommandSwitch(
                new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow (Far Detected)"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow (Center Detected)"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow (Close Detected)"), robot.autoDriveTrain),
                () -> robot.teamPropDetector.getTeamPropSide()
        );
    }
}
