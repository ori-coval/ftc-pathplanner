package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryPoses;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.ResetPixelCount;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class ScoreYellowClose extends SequentialCommandGroup {

    public static RobotControl robot = AutonomousOpMode.robot;

    public ScoreYellowClose() {
        addCommands(
                new ParallelCommandGroup(
                        getPreScoreArmPosition(),
                        getScoreYellowTrajectory()/*.andThen(resetPoseEstimate())*/
                ),
                new ConditionalCommand(
                        new DetectionSideCommandSwitch(
                                new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE_RED_CLOSE, false),
                                new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_FAR_RED_CLOSE, false),
                                new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_FRONT, true),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        new DetectionSideCommandSwitch(
                                new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE_BLUE_CLOSE, true),
                                new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_FAR_BLUE_CLOSE, true),
                                new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_FRONT_LOWER, true),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        () -> robot.allianceColor == AllianceColor.RED
                ),
                new WaitCommand(500),
                new CartridgeSetState(robot.cartridge, Cartridge.State.AUTONOMOUS_OPEN),
                new ResetPixelCount(robot),
                new WaitCommand(1000),
                getPreScoreArmPosition(),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS) //todo i really don't know how the cartridge works at this point
        );
    }

    private Command getPreScoreArmPosition() {
        return new ConditionalCommand(
                new ArmGetToPosition(robot, ArmPosition.SAFE_PLACE, true),
                new ArmGetToPosition(robot, ArmPosition.SCORING, robot.allianceColor == AllianceColor.BLUE),
                () -> robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
        );
    }

    private Command getScoreYellowTrajectory() {
        return new ConditionalCommand(
                new DetectionSideCommandSwitch(
                        new TrajectoryFollowerCommand(FAR_RED, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(CENTER_RED, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(CLOSE_RED, robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new DetectionSideCommandSwitch(
                        new TrajectoryFollowerCommand(FAR_BLUE, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(CENTER_BLUE, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(CLOSE_BLUE, robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private Command resetPoseEstimate() {
        return new ConditionalCommand(
                new ConditionalCommand(
                        new InstantCommand(() -> robot.autoDriveTrain.setPoseEstimate(new Pose2d(
                                TrajectoryPoses.realBackdropClosePoseRed.getX(),
                                TrajectoryPoses.realBackdropClosePoseRed.getY(),
                                robot.autoDriveTrain.getPoseEstimate().getHeading()
                        ))),
                        new InstantCommand(() -> robot.autoDriveTrain.setPoseEstimate(new Pose2d(
                                TrajectoryPoses.realBackdropClosePoseBlue.getX(),
                                TrajectoryPoses.realBackdropClosePoseBlue.getY(),
                                robot.autoDriveTrain.getPoseEstimate().getHeading()
                        ))),
                        () -> robot.allianceColor == AllianceColor.RED
                ),
                new InstantCommand(),
                () -> robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
        );
    }



    static final TrajectorySequence FAR_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_FAR_RED.end())
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(
                    new Pose2d(-52, -64, Math.toRadians(90)),
                    Math.toRadians(0),//Tangent
                    robot.trajectories.reduceVelocity(0.9),
                    robot.trajectories.reduceAcceleration(0.9)
            )
            .build();

    static final TrajectorySequence CENTER_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_CENTER_RED.end())
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(
                    new Pose2d(-51, -65, Math.toRadians(90)),
                    Math.toRadians(0), //Tangent
                    robot.trajectories.reduceVelocity(0.9),
                    robot.trajectories.reduceAcceleration(0.9)
            )
            .build();

    static final TrajectorySequence CLOSE_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_CLOSE_RED.end())
            .setTangent(Math.toRadians(-90))
            .splineToSplineHeading(
                    new Pose2d(-29, -55, Math.toRadians(90)),
                    Math.toRadians(-90), //Tangent
                    robot.trajectories.reduceVelocity(0.9),
                    robot.trajectories.reduceAcceleration(0.9)

            )
            .build();




    static final TrajectorySequence FAR_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_FAR_BLUE.end())
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(
                    new Pose2d(52, -64, Math.toRadians(90)),
                    Math.toRadians(180), //Tangent
                    robot.trajectories.reduceVelocity(0.9),
                    robot.trajectories.reduceAcceleration(0.9)
            )
            .build();

    static final TrajectorySequence CENTER_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_CENTER_BLUE.end())
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(
                    new Pose2d(52, -64, Math.toRadians(90)),
                    Math.toRadians(180), //Tangent
                    robot.trajectories.reduceVelocity(0.9),
                    robot.trajectories.reduceAcceleration(0.9)
            )
            .build();

    static final TrajectorySequence CLOSE_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_CLOSE_BLUE.end())
            .setTangent(Math.toRadians(270))
            .splineToSplineHeading(
                    new Pose2d(29, -55, Math.toRadians(90)),
                    Math.toRadians(270), //Tangent
                    robot.trajectories.reduceVelocity(0.9),
                    robot.trajectories.reduceAcceleration(0.9)

            )
            .build();

}
