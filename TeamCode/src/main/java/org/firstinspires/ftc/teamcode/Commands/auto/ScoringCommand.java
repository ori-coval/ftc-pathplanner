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
import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryPoses;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.ResetPixelCount;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class ScoringCommand extends SequentialCommandGroup {

    static RobotControl robot = AutonomousOpMode.robot;

    public ScoringCommand(Command scoringCommand, Command secondScoringCommand, RobotControl robot, Integer numOfCycle) {
        ScoringCommand.robot = robot;
        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryCommand(robot, numOfCycle),
                        new ConditionalCommand(
                                new IntakeRotate(robot.intake.roller, robot.intake.roller.EJECT_POWER).withTimeout(1500),
                                new InstantCommand(),
                                () -> numOfCycle != 0
                        ),
                        new WaitCommand(getWaitTime(robot, numOfCycle)).andThen(
                                new ConditionalCommand(
                                        new ArmGetToPosition(robot, ArmPosition.SCORING, robot.allianceColor == AllianceColor.RED),
                                        new ArmGetToPosition(robot, ArmPosition.SAFE_PLACE, true),
                                        () -> robot.teamPropDetector.getTeamPropSide() != DetectionSide.CLOSE & numOfCycle == 0
                                ),
                                new InstantCommand(() -> RotateTurretByPID.DEADLINE_FOR_TURRET = 700),//todo maybe will work with less time
                                new ConditionalCommand(
                                        new ConditionalCommand(
                                                new WaitCommand(650), //cycles
                                                new WaitCommand(600), //close yellow
                                                () -> numOfCycle != 0
                                        ),
                                        new InstantCommand(),
                                        () -> (numOfCycle != 0) || robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
                                ),
                                scoringCommand
                        )
                ),
                new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN),
                new ResetPixelCount(robot),
                new InstantCommand(() -> RotateTurretByPID.DEADLINE_FOR_TURRET = 2000),
                new WaitCommand(300)
// <- todo remove this when we need elevator               secondScoringCommand
        );
    }


    private long getWaitTime(RobotControl robot, int numOfCycle) {
        if(robot.allianceColor == AllianceColor.RED) {
            return numOfCycle == 0 ? 1700 : 1200;
        } else {
            return numOfCycle == 0 ? 1700 : 1700;
        }
    }

    private Command getTrajectoryCommand(RobotControl robot, int numOfCycle) {
        return new ConditionalCommand(
                        new ConditionalCommand(
                                new TrajectoryFollowerCommand(NORMAL_RED, robot.autoDriveTrain).andThen(resetPoseEstimateOnSide(robot)),
                                new TrajectoryFollowerCommand(FRONT_RED, robot.autoDriveTrain).andThen(resetPoseEstimateFront(robot)),
                                () -> robot.teamPropDetector.getTeamPropSide() != DetectionSide.CLOSE && numOfCycle == 0
                        ),
                        new ConditionalCommand(
                                new TrajectoryFollowerCommand(NORMAL_BLUE, robot.autoDriveTrain).andThen(resetPoseEstimateOnSide(robot)),
                                new ConditionalCommand(
                                        new TrajectoryFollowerCommand(FRONT_BLUE, robot.autoDriveTrain).andThen(resetPoseEstimateFront(robot)),
                                        new TrajectoryFollowerCommand(CYCLES_FRONT_BLUE, robot.autoDriveTrain).andThen(resetPoseEstimateFront(robot)),
                                        () -> numOfCycle == 0
                                ),
                                () -> robot.teamPropDetector.getTeamPropSide() != DetectionSide.CLOSE && numOfCycle == 0
                        ),
                        () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private Command resetPoseEstimateFront(RobotControl robot) {
        return new InstantCommand(() ->
                robot.autoDriveTrain.setPoseEstimate(new Pose2d(
                        robot.autoDriveTrain.getPoseEstimate().getX(),
                        TrajectoryPoses.realBackdropFront.getY(),
                        robot.autoDriveTrain.getPoseEstimate().getHeading()
                ))
        );
    }

    private Command resetPoseEstimateOnSide(RobotControl robot) {
        return new ConditionalCommand(
                new InstantCommand(() ->
                        robot.autoDriveTrain.setPoseEstimate(new Pose2d(
                                TrajectoryPoses.realBackdropFarPoseRed.getX(),
                                TrajectoryPoses.realBackdropFarPoseRed.getY(),
                                robot.autoDriveTrain.getPoseEstimate().getHeading()
                        ))
                ),
                new InstantCommand(() ->
                    robot.autoDriveTrain.setPoseEstimate(new Pose2d(
                            TrajectoryPoses.realBackdropFarPoseBlue.getX(),
                            TrajectoryPoses.realBackdropFarPoseBlue.getY(),
                            robot.autoDriveTrain.getPoseEstimate().getHeading()
                    ))
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    //Go to backdrop depending on alliance color.


    static final TrajectorySequence NORMAL_RED = robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.stackPoseRed)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(
                        new Pose2d(TrajectoryPoses.stackPoseRed.getX() + 3, -15, Math.toRadians(90)),
                        Math.toRadians(-90) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(-9, -40),
                        Math.toRadians(-90) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(-19, -59),
                        Math.toRadians(180), //Tangent
                        robot.trajectories.reduceVelocity(0.6),
                        robot.trajectories.reduceAcceleration(0.6)
                )
                .build();
    static final TrajectorySequence FRONT_RED = robot.autoDriveTrain.trajectorySequenceBuilder(LeaveSpikeMark.CLOSE_RED.end())
            .setTangent(Math.toRadians(-70))
            .splineToConstantHeading(
                    new Vector2d(TrajectoryPoses.stackPoseRed.getX() + 1, -25),
                    Math.toRadians(-95) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(-30, -30),
                    Math.toRadians(180) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(-42, -35),
                    Math.toRadians(-90), //Tangent
                    robot.trajectories.reduceVelocity(0.4),
                    robot.trajectories.reduceAcceleration(0.4)
            )
            .splineToConstantHeading(
                    new Vector2d(-42, -47),
                    Math.toRadians(-90), //Tangent
                    robot.trajectories.reduceVelocity(0.4),
                    robot.trajectories.reduceAcceleration(0.4)
            )
            .build();



    //BLUE


    static final TrajectorySequence NORMAL_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.stackPoseBlue)
            .setTangent(Math.toRadians(270))
            .splineToSplineHeading(
                    new Pose2d(TrajectoryPoses.stackPoseBlue.getX() + 3, -15, Math.toRadians(90)),
                    Math.toRadians(270) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(9, -40),
                    Math.toRadians(270) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(22, -52),
                    Math.toRadians(-30), //Tangent
                    robot.trajectories.reduceVelocity(0.6),
                    robot.trajectories.reduceAcceleration(0.6)
            )
            .build();
    static final TrajectorySequence FRONT_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(LeaveSpikeMark.CLOSE_BLUE.end())
            .setTangent(Math.toRadians(250))
            .splineToConstantHeading(
                    new Vector2d(TrajectoryPoses.stackPoseBlue.getX() + 1, -25),
                    Math.toRadians(275) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(30, -30),
                    Math.toRadians(0) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(46, -35),
                    Math.toRadians(270), //Tangent
                    robot.trajectories.reduceVelocity(0.4),
                    robot.trajectories.reduceAcceleration(0.4)
            )
            .splineToConstantHeading(
                    new Vector2d(46, -48),
                    Math.toRadians(270), //Tangent
                    robot.trajectories.reduceVelocity(0.4),
                    robot.trajectories.reduceAcceleration(0.4)
            )
            .build();
    static final TrajectorySequence CYCLES_FRONT_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.stackPoseBlue)
            .setTangent(Math.toRadians(250))
            .splineToConstantHeading(
                    new Vector2d(TrajectoryPoses.stackPoseBlue.getX() + 1, -25),
                    Math.toRadians(275) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(30, -30),
                    Math.toRadians(0) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(44, -35),
                    Math.toRadians(270), //Tangent
                    robot.trajectories.reduceVelocity(0.4),
                    robot.trajectories.reduceAcceleration(0.4)
            )
            .splineToConstantHeading(
                    new Vector2d(44, -51),
                    Math.toRadians(270), //Tangent
                    robot.trajectories.reduceVelocity(0.4),
                    robot.trajectories.reduceAcceleration(0.4)
            )
            .build();

}
