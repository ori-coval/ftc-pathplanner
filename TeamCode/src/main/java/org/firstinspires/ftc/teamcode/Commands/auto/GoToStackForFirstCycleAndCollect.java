package org.firstinspires.ftc.teamcode.Commands.auto;

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
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetLifterPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.InstantIntakeRotate;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class GoToStackForFirstCycleAndCollect extends SequentialCommandGroup {

    static RobotControl robot = AutonomousOpMode.robot;

    public GoToStackForFirstCycleAndCollect() {
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> RotateTurretByPID.DEADLINE_FOR_TURRET = 700),
                                new ParallelCommandGroup(
                                        getTrajectoryCommandPart1(),
                                        new ArmGetToPosition(robot, ArmPosition.SCORING, robot.allianceColor == AllianceColor.RED).andThen(
                                                new ArmGetToPosition(robot, ArmPosition.INTAKE, true)
                                        ),
                                        new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
                                ),
                                new InstantCommand(() -> RotateTurretByPID.DEADLINE_FOR_TURRET = 2000),
                                new ParallelCommandGroup(
                                        getTrajectoryCommandPart2(),
                                        new WaitCommand(300).andThen(
                                                new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN)
                                        ),
                                        new WaitCommand(1700).andThen(
                                                new InstantIntakeRotate(robot, robot.intake.roller.COLLECT_POWER),
                                                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL)
                                        )
                                )
                        ),
                        new ParallelCommandGroup(
                                getFrontTrajectory(),
                                new ConditionalCommand(
                                        new ArmGetToPosition(robot, ArmPosition.DRIVING_INTAKE, true),
                                        new ArmGetToPosition(robot, ArmPosition.INTAKE, true),
                                        () -> robot.allianceColor == AllianceColor.RED
                                ).andThen(
                                        new WaitCommand(1700),
                                        new ArmGetToPosition(robot, ArmPosition.INTAKE, true)
                                ),
                                new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN),
                                new WaitCommand(3000).andThen(
                                        new InstantIntakeRotate(robot, robot.intake.roller.COLLECT_POWER),
                                        new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL)
                                )
                        ),
                        () -> robot.teamPropDetector.getTeamPropSide() != DetectionSide.CLOSE
                ),
                new CollectFromStack(robot)
        );
    }

    private Command getFrontTrajectory() {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(
                        FRONT_RED,
                        robot.autoDriveTrain
                ),
                new TrajectoryFollowerCommand(
                        FRONT_BLUE,
                        robot.autoDriveTrain
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private Command getTrajectoryCommandPart1() {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(
                        FIRST_PART_RED,
                        robot.autoDriveTrain
                ),
                new ConditionalCommand(
                        new TrajectoryFollowerCommand( //FAR
                                FIRST_PART_FAR_BLUE,
                                robot.autoDriveTrain
                        ),
                        new TrajectoryFollowerCommand( //CENTER
                                FIRST_PART_CENTER_BLUE,
                                robot.autoDriveTrain
                        ),
                        () -> robot.teamPropDetector.getTeamPropSide() == DetectionSide.FAR
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private Command getTrajectoryCommandPart2() {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(
                        SECOND_PART_RED,
                        robot.autoDriveTrain
                ),
                new ConditionalCommand(
                        new TrajectoryFollowerCommand( //FAR
                                SECOND_PART_FAR_BLUE,
                                robot.autoDriveTrain
                        ),
                        new TrajectoryFollowerCommand( //CENTER
                                SECOND_PART_CENTER_BLUE,
                                robot.autoDriveTrain
                        ),
                        () -> robot.teamPropDetector.getTeamPropSide() == DetectionSide.FAR
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }


    static final TrajectorySequence FIRST_PART_RED = robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.realBackdropFarPoseRed)
            .setTangent(Math.toRadians(0))
            .splineToConstantHeading(
                    new Vector2d(-7, -40),
                    Math.toRadians(90) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(Trajectories.stackPoseRed.getX() + 3, -15),
                    Math.toRadians(90) //Tangent
            )
            .build();
    static final TrajectorySequence FRONT_RED = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringCommand.YELLOW_FRONT_RED.end())
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(
                    new Vector2d(-40, -46),
                    Math.toRadians(90)
            )
            .splineToConstantHeading(
                    new Vector2d(-17, -25),
                    Math.toRadians(45) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(Trajectories.stackPoseRed.getX() + 3, 0),
                    Math.toRadians(90) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(
                            Trajectories.stackPoseRed.getX() + 2,
                            Trajectories.stackPoseRed.getY() + 5
                    ),
                    Math.toRadians(90), //Tangent
                    Trajectories.reduceVelocity(0.7),
                    Trajectories.reduceAcceleration(0.7)

            )
            .build();
    static final TrajectorySequence SECOND_PART_RED = robot.autoDriveTrain.trajectorySequenceBuilder(FIRST_PART_RED.end())
            .splineToConstantHeading(
                    new Vector2d(Trajectories.stackPoseRed.getX(), 48),
                    Math.toRadians(90) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(
                            Trajectories.stackPoseRed.getX(),
                            Trajectories.stackPoseRed.getY() + 7
                    ),
                    Math.toRadians(90), //Tangent
                    Trajectories.reduceVelocity(0.7),
                    Trajectories.reduceAcceleration(0.7)
            )
            .build();


    //BLUE


    static final TrajectorySequence FIRST_PART_CENTER_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.realBackdropFarPoseBlue)
            .setTangent(Math.toRadians(180))
            .splineToConstantHeading(
                    new Vector2d(7, -40),
                    Math.toRadians(90) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(Trajectories.stackPoseBlue.getX() - 3, -15),
                    Math.toRadians(90) //Tangent
            )
            .build();
    static final TrajectorySequence SECOND_PART_CENTER_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(FIRST_PART_CENTER_BLUE.end())
            .splineToConstantHeading(
                    new Vector2d(Trajectories.stackPoseBlue.getX() - 3, 48),
                    Math.toRadians(90) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(
                            Trajectories.stackPoseBlue.getX() - 3,
                            Trajectories.stackPoseBlue.getY() - 2
                    ),
                    Math.toRadians(90), //Tangent
                    Trajectories.reduceVelocity(0.7),
                    Trajectories.reduceAcceleration(0.7)
            )
            .build();

    static final TrajectorySequence FIRST_PART_FAR_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.realBackdropFarPoseBlue)
            .setTangent(Math.toRadians(180))
            .splineToConstantHeading(
                    new Vector2d(7, -40),
                    Math.toRadians(90) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(Trajectories.stackPoseBlue.getX() - 3, -15),
                    Math.toRadians(90) //Tangent
            )
            .build();
    static final TrajectorySequence SECOND_PART_FAR_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(FIRST_PART_CENTER_BLUE.end())
            .splineToConstantHeading(
                    new Vector2d(Trajectories.stackPoseBlue.getX() - 5, 48),
                    Math.toRadians(90) //Tangent
            )
            .splineToConstantHeading(
                    new Vector2d(
                            Trajectories.stackPoseBlue.getX() - 4,
                            Trajectories.stackPoseBlue.getY() - 1
                    ),
                    Math.toRadians(90), //Tangent
                    Trajectories.reduceVelocity(0.7),
                    Trajectories.reduceAcceleration(0.7)
            )
            .build();


    static final TrajectorySequence FRONT_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringCommand.YELLOW_FRONT_BLUE.end())
            .setTangent(Math.toRadians(100))
            .splineToConstantHeading(
                    new Vector2d(Trajectories.stackPoseBlue.getX() - 3, -25),
                    Math.toRadians(90)
            )
            .splineToConstantHeading(
                    new Vector2d(Trajectories.stackPoseBlue.getX() - 3, 0),
                    Math.toRadians(90)
            )
            .splineToConstantHeading(
                    new Vector2d(
                            Trajectories.stackPoseBlue.getX() - 2,
                            Trajectories.stackPoseBlue.getY() - 1
                    ),
                    Math.toRadians(90), //Tangent
                    Trajectories.reduceVelocity(0.7),
                    Trajectories.reduceAcceleration(0.7)

            )
            .build();

}
