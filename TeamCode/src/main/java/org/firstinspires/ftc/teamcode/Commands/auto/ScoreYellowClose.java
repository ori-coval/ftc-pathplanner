package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
                        getScoreYellowTrajectory().andThen(resetPoseEstimate())
                ),
                new DetectionSideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_FAR, robot.allianceColor == AllianceColor.RED),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_FRONT, true),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN),
                new ResetPixelCount(robot),
                new WaitCommand(500),
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
                    new Pose2d(-53, -64, Math.toRadians(90)),
                    Math.toRadians(-45) //Tangent
            )
            .build();

    static final TrajectorySequence CENTER_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_CENTER_RED.end())
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(
                    new Pose2d(-53, -64, Math.toRadians(90)),
                    Math.toRadians(-45) //Tangent
            )
            .build();

    static final TrajectorySequence CLOSE_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_CLOSE_RED.end())
            .setTangent(Math.toRadians(-90))
            .splineToSplineHeading(
                    new Pose2d(-33, -55, Math.toRadians(90)),
                    Math.toRadians(-90) //Tangent
            )
            .build();




    static final TrajectorySequence FAR_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_FAR_BLUE.end())
            .setTangent(Math.toRadians(180))
            .splineToConstantHeading(
                    new Vector2d(20, -20),
                    Math.toRadians(180) //Tangent
            )
            .splineToSplineHeading(
                    new Pose2d(10, -30, Math.toRadians(90)),
                    Math.toRadians(270) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(20, -40),
                    Math.toRadians(0) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(60, -64),
                    Math.toRadians(270) //Tangent
            )
            .build();

    static final TrajectorySequence CENTER_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_CENTER_BLUE.end())
            .splineToSplineHeading(
                    new Pose2d(15, -27, Math.toRadians(90)),
                    Math.toRadians(270) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(60, -64),
                    Math.toRadians(270) //Tangent
            )
            .build();

    static final TrajectorySequence CLOSE_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.CLOSE_CLOSE_BLUE.end())
            .setTangent(Math.toRadians(225))
            .splineToConstantHeading(
                    new Vector2d(20, -20),
                    Math.toRadians(-20) //Tangent
            )
            .splineToSplineHeading(
                    new Pose2d(60, -64, Math.toRadians(90)),
                    Math.toRadians(270) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(53, -64),
                    Math.toRadians(180) //Tangent
            )
            .build();

}
