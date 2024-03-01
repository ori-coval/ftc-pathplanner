package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

public class ScoringPurplePixel extends ParallelCommandGroup {

    public ScoringPurplePixel(RobotControl robot) {
        super(
                new ConditionalCommand(
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Far Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Center Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Close Detected)"), robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple (Far Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple (Center Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple (Close Detected)"), robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        () -> robot.robotSide == AllianceSide.FAR
                ),
                new WaitCommand(300).andThen(new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL, false)),
                new WaitCommand(1500).andThen(
                        new IntakeRotate(robot.intake.roller, robot.intake.roller.COLLECT_POWER).withTimeout(2000)
                )
        );
    }
}
