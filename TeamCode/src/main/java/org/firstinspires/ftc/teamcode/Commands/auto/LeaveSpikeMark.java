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
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class LeaveSpikeMark extends ParallelCommandGroup {

    static RobotControl robot = AutonomousOpMode.robot;

    public LeaveSpikeMark() {
        addCommands(
                getTrajectoryCommand(robot),
                new WaitCommand(1200).andThen(
                        new ArmGetToPosition(robot, ArmPosition.INTAKE, true),
                        new WaitCommand(300),
                        new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
                )
        );
    }


    public static Command getTrajectoryCommand(RobotControl robot) {
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


    //This trajectory is meant to drive to stack depending on the prop detected and the alliance color.

    static final TrajectorySequence FAR_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_FAR_RED.end())
            .setTangent(Math.toRadians(75))
            .splineToLinearHeading(
                    new Pose2d(
                            TrajectoryPoses.stackPoseRed.getX(),
                            TrajectoryPoses.stackPoseRed.getY() - 7,
                            TrajectoryPoses.stackPoseRed.getHeading()
                    ),
                    Math.toRadians(-20) //Tangent
            )
            .build();

    static final TrajectorySequence CENTER_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_CENTER_RED.end())
            .setTangent(Math.toRadians(35))
            .splineToConstantHeading(
                    new Vector2d(-15, 42),
                    Math.toRadians(35)
            )
            .splineToSplineHeading(
                    new Pose2d(-10, 47, Math.toRadians(90)),
                    Math.toRadians(45) //Tangent
            )
            .build();

    static final TrajectorySequence CLOSE_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_CLOSE_RED.end())
            .setTangent(Math.toRadians(60))
            .splineToLinearHeading(
                    new Pose2d(-18, 43, Math.toRadians(90)),
                    Math.toRadians(30) //Tangent
            )
            .build();





    static final TrajectorySequence FAR_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_FAR_BLUE.end())
            .setTangent(Math.toRadians(105))
            .splineToLinearHeading(
                    new Pose2d(
                            TrajectoryPoses.stackPoseBlue.getX(),
                            TrajectoryPoses.stackPoseBlue.getY() - 4,
                            TrajectoryPoses.stackPoseBlue.getHeading()
                    ),
                    Math.toRadians(160) //Tangent
            )
            .build();

    static final TrajectorySequence CENTER_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_CENTER_BLUE.end())
            .setTangent(Math.toRadians(145))
            .splineToConstantHeading(
                    new Vector2d(15, 42),
                    Math.toRadians(145)
            )
            .splineToSplineHeading(
                    new Pose2d(10, 47, Math.toRadians(90)),
                    Math.toRadians(135) //Tangent
            )
            .build();

    static final TrajectorySequence CLOSE_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_CLOSE_BLUE.end())
            .setTangent(Math.toRadians(120))
            .splineToLinearHeading(
                    new Pose2d(18, 43, Math.toRadians(90)),
                    Math.toRadians(150) //Tangent
            )
            .build();

}
