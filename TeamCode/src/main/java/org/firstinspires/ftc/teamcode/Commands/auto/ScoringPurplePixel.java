package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class ScoringPurplePixel extends ParallelCommandGroup {

    public static RobotControl robot = AutonomousOpMode.robot;

    public ScoringPurplePixel() {
        addCommands(
                getTrajectoryCommand().andThen(
                        new ConditionalCommand(
                                new WaitCommand(300),
                                new InstantCommand(),
                                () -> robot.teamPropDetector.getTeamPropSide() == DetectionSide.CENTER && robot.allianceColor == AllianceColor.BLUE
                        )
                ), //1800, 2000
                new ConditionalCommand( //blue far
                        new WaitCommand(1800).andThen(
                                new IntakeRotate(robot.intake.roller, robot.intake.roller.PURPLE_PIXEL_FAR_BLUE_POWER).withTimeout(420)
                        ), //else
                        new ConditionalCommand(
                                new WaitCommand(2000).andThen(
                                        new ConditionalCommand(
                                                new IntakeRotate(robot.intake.roller, robot.intake.roller.PURPLE_PIXEL_FAR_RED_POWER).withTimeout(420),
                                                new IntakeRotate(robot.intake.roller, robot.intake.roller.PURPLE_PIXEL_FAR_BLUE_POWER).withTimeout(420),
                                                () -> robot.allianceColor == AllianceColor.RED
                                        )
                                ),
                                new WaitCommand(2000).andThen(
                                        new ConditionalCommand(
                                                new IntakeRotate(robot.intake.roller, robot.intake.roller.PURPLE_PIXEL_CLOSE_RED_POWER).withTimeout(200),
                                                new IntakeRotate(robot.intake.roller, robot.intake.roller.PURPLE_PIXEL_CLOSE_BLUE_POWER).withTimeout(250),
                                                () -> robot.allianceColor == AllianceColor.RED
                                        )
                                ),
                                () -> robot.robotSide == AllianceSide.FAR
                        ),
                        () -> robot.allianceColor == AllianceColor.BLUE && robot.teamPropDetector.getTeamPropSide() == DetectionSide.FAR && robot.robotSide == AllianceSide.FAR
                ),
                new WaitCommand(300).andThen(
                        new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL_1, true),
                        new WaitCommand(2200),
                        new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL_2, true)
                )
        );
    }

    private Command getTrajectoryCommand() {
        return new ConditionalCommand(
                new ConditionalCommand(
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(FAR_FAR_RED, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(FAR_CENTER_RED, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(FAR_CLOSE_RED, robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(CLOSE_FAR_RED, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(CLOSE_CENTER_RED, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(CLOSE_CLOSE_RED, robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        () -> robot.robotSide == AllianceSide.FAR
                ),
                new ConditionalCommand(
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(FAR_FAR_BLUE, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(FAR_CENTER_BLUE, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(FAR_CLOSE_BLUE, robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(CLOSE_FAR_BLUE, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(CLOSE_CENTER_BLUE, robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(CLOSE_CLOSE_BLUE, robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        () -> robot.robotSide == AllianceSide.FAR
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );

    }

        //(Robot Side)_(Detection Side)
    static final TrajectorySequence FAR_FAR_RED = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
                .setTangent(Math.toRadians(30))
                .splineToSplineHeading(
                        new Pose2d(-40, 50, Math.toRadians(60)),
                        Math.toRadians(30) //Tangent
                )
                .splineToSplineHeading(
                        new Pose2d(-30, 50, Math.toRadians(45)),
                        Math.toRadians(-30) //Tangent
                ).build();
    static final TrajectorySequence FAR_CENTER_RED = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .splineToSplineHeading(
                    new Pose2d(-40, 37, Math.toRadians(60)),
                    Math.toRadians(0) //Tangent
            )
            .splineToSplineHeading(
                    new Pose2d(-24, 37, Math.toRadians(40)),
                    Math.toRadians(0)) //Tangent
            .build();
    static final TrajectorySequence FAR_CLOSE_RED = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .splineToSplineHeading(
                    new Pose2d(-50, robot.startPose.getY(), Math.toRadians(45)),
                    Math.toRadians(0) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(-34, 28),
                    Math.toRadians(-45) //Tangent
            )
            .build();
    static final TrajectorySequence CLOSE_FAR_RED = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .splineToConstantHeading(
                    new Vector2d(-41, -24),
                    Math.toRadians(0) //Tangent
            )
            .build();
    static final TrajectorySequence CLOSE_CENTER_RED = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .splineToConstantHeading(
                    new Vector2d(-32, -13),
                    Math.toRadians(0) //Tangent
            )
            .build();
    static final TrajectorySequence CLOSE_CLOSE_RED = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .setTangent(Math.toRadians(0))
            .splineToSplineHeading(
                    new Pose2d(-50, -16, Math.toRadians(75)),
                    Math.toRadians(0) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(-36, -12),
                    Math.toRadians(40) //Tangent
            )
            .build();




    //(Robot Side)_(Detection Side)
    static final TrajectorySequence FAR_FAR_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .setTangent(Math.toRadians(150))
            .splineToSplineHeading(
                    new Pose2d(50, 50, Math.toRadians(120)),
                    Math.toRadians(150) //Tangent
            )
            .splineToSplineHeading(
                    new Pose2d(35, 49, Math.toRadians(135)),
                    Math.toRadians(210) //Tangent
            ).build();
    static final TrajectorySequence FAR_CENTER_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .splineToSplineHeading(
                    new Pose2d(40, 37, Math.toRadians(120)),
                    Math.toRadians(180) //Tangent
            )
            .splineToSplineHeading(
                    new Pose2d(26, 37, Math.toRadians(140)),
                    Math.toRadians(180)) //Tangent
            .build();
    static final TrajectorySequence FAR_CLOSE_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .splineToSplineHeading(
                    new Pose2d(50, robot.startPose.getY(), Math.toRadians(135)),
                    Math.toRadians(180) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(34, 27),
                    Math.toRadians(135) //Tangent
            )
            .build();

    //CLOSE
    static final TrajectorySequence CLOSE_FAR_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .splineToConstantHeading(
                    new Vector2d(41, -24),
                    Math.toRadians(180) //Tangent
            )
            .build();
    static final TrajectorySequence CLOSE_CENTER_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .splineToConstantHeading(
                    new Vector2d(32, -15),
                    Math.toRadians(180) //Tangent
            )
            .build();
    static final TrajectorySequence CLOSE_CLOSE_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(robot.startPose)
            .setTangent(Math.toRadians(180))
            .splineToSplineHeading(
                    new Pose2d(50, -16, Math.toRadians(105)),
                    Math.toRadians(180) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(37, -11),
                    Math.toRadians(140) //Tangent
            )
            .build();
}
