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

    public static RobotControl robot;

    public ScoreYellowClose(RobotControl robot) {
        ScoreYellowClose.robot = robot;
        addCommands(
                new ParallelCommandGroup(
                        getPreScorePosition(),
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
                getPreScorePosition(),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS) //todo i really don't know how the cartridge works at this point
        );
    }

    private Command getPreScorePosition() {
        return new ConditionalCommand(
                new ArmGetToPosition(robot, ArmPosition.SAFE_PLACE, true),
                new ArmGetToPosition(robot, ArmPosition.SCORING, robot.allianceColor == AllianceColor.BLUE),
                () -> robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
        );
    }

    private Command getScoreYellowTrajectory() {
        return new ConditionalCommand(
                new DetectionSideCommandSwitch(
                        new TrajectoryFollowerCommand(TrajectoriesRed.FAR.trajectory, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(TrajectoriesRed.CENTER.trajectory, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(TrajectoriesRed.CLOSE.trajectory, robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new DetectionSideCommandSwitch(
                        new TrajectoryFollowerCommand(TrajectoriesBlue.FAR.trajectory, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(TrajectoriesBlue.CENTER.trajectory, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(TrajectoriesBlue.CLOSE.trajectory, robot.autoDriveTrain),
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


    public enum TrajectoriesRed {

        FAR(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesRed.CLOSE_FAR.trajectory.end())
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(-20, -20),
                        Math.toRadians(0) //Tangent
                )
                .splineToSplineHeading(
                        new Pose2d(-10, -30, Math.toRadians(90)),
                        Math.toRadians(-90) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(-20, -40),
                        Math.toRadians(180) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(-60, -55),
                        Math.toRadians(-90) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(-53, -64),
                        Math.toRadians(0) //Tangent
                )
                .build()
        ),

        CENTER(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesRed.CLOSE_CENTER.trajectory.end())
                .splineToSplineHeading(
                        new Pose2d(-15, -27, Math.toRadians(90)),
                        Math.toRadians(-90) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(-60, -55),
                        Math.toRadians(-90) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(-53, -64),
                        Math.toRadians(0) //Tangent
                )
                .build()
        ),

        CLOSE(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesRed.CLOSE_CLOSE.trajectory.end())
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(
                        new Vector2d(-20, -20),
                        Math.toRadians(220) //Tangent
                )
                .splineToSplineHeading(
                        new Pose2d(-33, -58, Math.toRadians(90)),
                        Math.toRadians(-90) //Tangent
                )
                .build()
        );

        final TrajectorySequence trajectory;

        TrajectoriesRed(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }


    }

    public enum TrajectoriesBlue {

        FAR(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesBlue.CLOSE_FAR.trajectory.end())
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
                .build()
        ),

        CENTER(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesBlue.CLOSE_CENTER.trajectory.end())
                .splineToSplineHeading(
                        new Pose2d(15, -27, Math.toRadians(90)),
                        Math.toRadians(270) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(60, -64),
                        Math.toRadians(270) //Tangent
                )
                .build()
        ),

        CLOSE(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesBlue.CLOSE_CLOSE.trajectory.end())
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
                .build()
        );

        final TrajectorySequence trajectory;

        TrajectoriesBlue(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }


    }

}
