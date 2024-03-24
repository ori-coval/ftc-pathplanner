package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryPoses;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class Parking extends ParallelCommandGroup {
    static RobotControl robot = AutonomousOpMode.robot;
    public Parking() {
        addCommands(
                new ConditionalCommand(
                        getFarTrajectory(),
                        getCloseTrajectory(),
                        () -> robot.robotSide == AllianceSide.FAR
                ), //to allow arm to get back in
                new WaitCommand(500).andThen(
                        new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS),
                        new ArmGetToPosition(robot, ArmPosition.INTAKE, true)
                )
        );
    }

    private Command getFarTrajectory() {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(FAR_RED, robot.autoDriveTrain),
                new TrajectoryFollowerCommand(FAR_BLUE, robot.autoDriveTrain),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }


    private Command getCloseTrajectory() {
        return new ConditionalCommand(
                new ConditionalCommand(
                        new TrajectoryFollowerCommand(CLOSE_FRONT_RED, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(CLOSE_RED, robot.autoDriveTrain),
                        () ->  robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
                ),
                new ConditionalCommand(
                        new TrajectoryFollowerCommand(CLOSE_FRONT_BLUE, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(CLOSE_BLUE, robot.autoDriveTrain),
                        () ->  robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }



    static final TrajectorySequence FAR_RED = robot.autoDriveTrain.trajectorySequenceBuilder(
            new Pose2d(
                    ScoringCommand.FRONT_RED.end().getX(),
                    TrajectoryPoses.realBackdropFront.getY(),
                    ScoringCommand.FRONT_RED.end().getHeading()
            ))
            .forward(4)
            .build();

    static final TrajectorySequence CLOSE_RED = robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.realBackdropClosePoseRed)
            .setTangent(180)
            .splineToConstantHeading(
                    new Vector2d(-60, -50),
                    Math.toRadians(90)
            )
            .build();

    static final TrajectorySequence CLOSE_FRONT_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoreYellowClose.CLOSE_RED.end())
            .splineToConstantHeading(
                    new Vector2d(-33, -50),
                    Math.toRadians(90)
            )
            .build();






    static final TrajectorySequence FAR_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(
            new Pose2d(
                    ScoringCommand.FRONT_BLUE.end().getX(),
                    TrajectoryPoses.realBackdropFront.getY(),
                    ScoringCommand.FRONT_BLUE.end().getHeading()
            ))
            .forward(4)
            .build();

    static final TrajectorySequence CLOSE_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.realBackdropClosePoseBlue)
            .setTangent(0)
            .splineToConstantHeading(
                    new Vector2d(60, -50),
                    Math.toRadians(90)
            )
            .build();

    static final TrajectorySequence CLOSE_FRONT_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoreYellowClose.CLOSE_BLUE.end())
            .splineToConstantHeading(
                    new Vector2d(33, -50),
                    Math.toRadians(90)
            )
            .build();

}
