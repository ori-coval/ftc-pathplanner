package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

public class ScoringPurplePixel extends ParallelCommandGroup {

    public ScoringPurplePixel(RobotControl robot) {
        addCommands(
                new ConditionalCommand(
                        farCommand(robot),
                        closeCommand(robot),
                        () -> robot.robotSide == AllianceSide.FAR
                ),
                new WaitCommand(300).andThen(new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL, false)),
                new WaitCommand(1000).andThen(
                        new IntakeRotate(robot.intake.roller, robot.intake.roller.COLLECT_POWER).withTimeout(2000)
                )
        );
    }

    private Command farCommand(RobotControl robot) {
        return new ConditionalCommand(
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple Left"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple Center"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple Right"), robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple Right"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple Center"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple Left"), robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private Command closeCommand(RobotControl robot) {
        return new ConditionalCommand(
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple Left"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple Center"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple Right"), robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple Right"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple Center"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple Left"), robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

}
