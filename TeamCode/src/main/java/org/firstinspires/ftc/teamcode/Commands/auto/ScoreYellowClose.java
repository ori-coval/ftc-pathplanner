package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.Side;

public class ScoreYellowClose extends SequentialCommandGroup {
    public ScoreYellowClose(RobotControl robot) {
        addCommands(
//                new ArmGetToPosition(robot, ArmPosition.SCORING, robotSide(robot)),
                scoreYellowTrajectory(robot),
                new InstantCommand(() -> {
                    robot.telemetry.addData("RobotSide", robotSide(robot));
                    robot.telemetry.addLine(String.valueOf(robot.teamPropDetector.getTeamPropSide()));
                    robot.telemetry.update();
                })

        );
    }

    private boolean robotSide(RobotControl robot) {
        return (robot.allianceColor == AllianceColor.RED) ?
                robot.teamPropDetector.getTeamPropSide() == Side.LEFT :
                robot.teamPropDetector.getTeamPropSide() != Side.RIGHT;
    }

    private Command scoreYellowTrajectory(RobotControl robot) {
        return new ConditionalCommand(
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow Left"), robot.driveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow Center"), robot.driveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow Right"), robot.driveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow Right"), robot.driveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow Center"), robot.driveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Yellow Left"), robot.driveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }
}
