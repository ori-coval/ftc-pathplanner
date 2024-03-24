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
                new WaitCommand(200).andThen(
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
                    TrajectoryPoses.stackPoseRed,
                    Math.toRadians(45) //Tangent
            )
            .build();

    static final TrajectorySequence CENTER_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_CENTER_RED.end())
            .setTangent(Math.toRadians(40))
            .splineToSplineHeading(
                    new Pose2d(TrajectoryPoses.stackPoseRed.getX(), 47, Math.toRadians(90)),
                    Math.toRadians(90) //Tangent
            )
            .splineToSplineHeading(
                    TrajectoryPoses.stackPoseRed,
                    Math.toRadians(90) //Tangent
            )
            .build();

    static final TrajectorySequence CLOSE_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_CLOSE_RED.end())
            .setTangent(Math.toRadians(45))
            .splineToLinearHeading(
                    new Pose2d(-18, 40, Math.toRadians(90)),
                    Math.toRadians(30) //Tangent
            )
            .build();





    static final TrajectorySequence FAR_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_FAR_BLUE.end())
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(
                        TrajectoryPoses.stackPoseBlue,
                        Math.toRadians(135) //Tangent
                )
                .build();
    static final TrajectorySequence CENTER_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_CENTER_BLUE.end())
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(
                        new Pose2d(TrajectoryPoses.stackPoseBlue.getX(), 47, Math.toRadians(90)),
                        Math.toRadians(90) //Tangent
                )
                .splineToSplineHeading(
                        TrajectoryPoses.stackPoseRed,
                        Math.toRadians(90) //Tangent
                )
                .build();
    static final TrajectorySequence CLOSE_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.FAR_CLOSE_BLUE.end())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        new Vector2d(22, 40),
                        Math.toRadians(135) //Tangent
                )
                .splineToSplineHeading(
                        TrajectoryPoses.stackPoseBlue,
                        Math.toRadians(90) //Tangent
                )
                .build();

}
