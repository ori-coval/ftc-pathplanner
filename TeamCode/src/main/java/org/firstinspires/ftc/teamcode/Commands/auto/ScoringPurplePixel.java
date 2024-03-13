package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

public class ScoringPurplePixel extends ParallelCommandGroup {

    public static RobotControl robot;

    public ScoringPurplePixel(RobotControl robot) {
        ScoringPurplePixel.robot = robot;
        addCommands(
                getTrajectoryCommand(),
                new WaitCommand(2000).andThen(new IntakeRotate(robot.intake.roller, robot.intake.roller.COLLECT_POWER).withTimeout(300)),
                new WaitCommand(300).andThen(
                        new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL, true)
                )
        );
    }

    private Command getTrajectoryCommand() {
        return new ConditionalCommand(
                new ConditionalCommand(
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(TrajectoriesRed.FAR_FAR_PURPLE.trajectory, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(TrajectoriesRed.FAR_CENTER_PURPLE.trajectory, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(TrajectoriesRed.FAR_CLOSE_PURPLE.trajectory, robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        new DetectionSideCommandSwitch( //todo
                                new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple (Far Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple (Center Detected) Red"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple (Close Detected) Red"), robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        () -> robot.robotSide == AllianceSide.FAR
                ),
                new ConditionalCommand(
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(TrajectoriesBlue.FAR_FAR_PURPLE.trajectory, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(TrajectoriesBlue.FAR_CENTER_PURPLE.trajectory, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(TrajectoriesBlue.FAR_CLOSE_PURPLE.trajectory, robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        new DetectionSideCommandSwitch( //todo
                                new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Far Detected) Blue"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Center Detected) Blue"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Close Detected) Blue"), robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        () -> robot.robotSide == AllianceSide.FAR
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );

    }

    public enum TrajectoriesRed {
        FAR_FAR_PURPLE(robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
                .splineToSplineHeading(
                        new Pose2d(-40, 50, Math.toRadians(60)),
                        Math.toRadians(0) //Tangent
                )
                .splineToSplineHeading(
                        new Pose2d(-30, 50, Math.toRadians(60)),
                        Math.toRadians(0) //Tangent
                )
                .build()
        ),

        FAR_CENTER_PURPLE(robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
                .splineToSplineHeading(
                        new Pose2d(-40, 37, Math.toRadians(60)),
                        Math.toRadians(0) //Tangent
                )
                .splineToSplineHeading(
                        new Pose2d(-25, 37, Math.toRadians(40)),
                        Math.toRadians(0)) //Tangent
                .build()
        ),

        FAR_CLOSE_PURPLE(robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
                .splineToSplineHeading(
                        new Pose2d(-50, robot.startPose.getY(), Math.toRadians(45)),
                        Math.toRadians(0) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(-34, 29),
                        Math.toRadians(-45) //Tangent
                )
                .build()
        );

        final TrajectorySequence trajectory;

        TrajectoriesRed(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        };

    }



    public enum TrajectoriesBlue {
        FAR_FAR_PURPLE(robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
                .splineToSplineHeading(
                        new Pose2d(40, 50, Math.toRadians(120)),
                        Math.toRadians(180) //Tangent
                )
                .splineToSplineHeading(
                        new Pose2d(30, 50, Math.toRadians(120)),
                        Math.toRadians(180) //Tangent
                )
                .build()
        ),

        FAR_CENTER_PURPLE(robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
                .splineToSplineHeading(
                        new Pose2d(40, 37, Math.toRadians(120)),
                        Math.toRadians(180) //Tangent
                )
                .splineToSplineHeading(
                        new Pose2d(25, 37, Math.toRadians(140)),
                        Math.toRadians(180)) //Tangent
                .build()
        ),

        FAR_CLOSE_PURPLE(robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
                .splineToSplineHeading(
                        new Pose2d(50, robot.startPose.getY(), Math.toRadians(135)),
                        Math.toRadians(180) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(36, 26),
                        Math.toRadians(225) //Tangent
                )
                .build()
        );


        final TrajectorySequence trajectory;

        TrajectoriesBlue(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }
    }


}
