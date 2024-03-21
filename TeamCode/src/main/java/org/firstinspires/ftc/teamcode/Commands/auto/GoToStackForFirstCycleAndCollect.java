package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryPoses;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetLifterPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.InstantIntakeRotate;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class GoToStackForFirstCycleAndCollect extends SequentialCommandGroup {

    static RobotControl robot;

    public GoToStackForFirstCycleAndCollect(RobotControl robot) {
        GoToStackForFirstCycleAndCollect.robot = robot;
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        getTrajectoryCommandPart1(),
                                        new ArmGetToPosition(robot, ArmPosition.SCORING_AUTO, robot.allianceColor == AllianceColor.RED).andThen(
                                                new ArmGetToPosition(robot, ArmPosition.INTAKE, true)
                                        ),
                                        new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
                                ),
                                new ParallelCommandGroup(
                                        getTrajectoryCommandPart2(robot),
                                        new WaitCommand(300).andThen(
                                                new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN)
                                        ),
                                        new WaitCommand(500).andThen(
                                                new InstantIntakeRotate(robot, robot.intake.roller.COLLECT_POWER),
                                                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL)
                                        )
                                )
                        ),
                        new ParallelCommandGroup(
                                getFrontTrajectory(),
                                new ArmGetToPosition(robot, ArmPosition.INTAKE, true),
                                new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN),
                                new WaitCommand(700).andThen(
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
                        TrajectoriesRed.FRONT.trajectory,
                        robot.autoDriveTrain
                ),
                new TrajectoryFollowerCommand(
                        TrajectoriesBlue.FRONT.trajectory,
                        robot.autoDriveTrain
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private Command getTrajectoryCommandPart1() {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(
                        TrajectoriesRed.FIRST_PART.trajectory,
                        robot.autoDriveTrain
                ),
                new TrajectoryFollowerCommand(
                        TrajectoriesBlue.FIRST_PART.trajectory,
                        robot.autoDriveTrain
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private Command getTrajectoryCommandPart2(RobotControl robot) {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(
                        TrajectoriesRed.SECOND_PART.trajectory,
                        robot.autoDriveTrain
                ),
                new TrajectoryFollowerCommand(
                        TrajectoriesBlue.SECOND_PART.trajectory,
                        robot.autoDriveTrain
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    public enum TrajectoriesRed {

        FIRST_PART(robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.realBackdropFarPoseRed)
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(
                        new Pose2d(-9, -40, Math.toRadians(90)),
                        Math.toRadians(90) //Tangent
                )
                .splineToLinearHeading(
                        new Pose2d(-36, -15, Math.toRadians(90)),
                        Math.toRadians(92) //Tangent
                )
                .build()
        ),
        FRONT(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringCommand.TrajectoriesRed.FRONT.trajectory.end())
                .setTangent(Math.toRadians(80))
                .splineToConstantHeading(
                        new Vector2d(-36, 58),
                        Math.toRadians(90) //Tangent
                )
                .build()
        ),
        SECOND_PART(robot.autoDriveTrain.trajectorySequenceBuilder(FIRST_PART.trajectory.end())
                .splineToConstantHeading(
                        new Vector2d(-36, 48),
                        Math.toRadians(90) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(-36, 58),
                        Math.toRadians(90), //Tangent
                        robot.trajectories.reduceVelocity(0.4),
                        robot.trajectories.reduceAcceleration(0.4)
                )
                .build()
        );

        final TrajectorySequence trajectory;

        TrajectoriesRed(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }

    }


    public enum TrajectoriesBlue {

        FIRST_PART(robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.realBackdropFarPoseBlue)
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(
                        new Pose2d(9, -40, Math.toRadians(90)),
                        Math.toRadians(90) //Tangent
                )
                .splineToLinearHeading(
                        new Pose2d(36, -15, Math.toRadians(90)),
                        Math.toRadians(88) //Tangent
                )
                .build()
        ),

        FRONT(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringCommand.TrajectoriesBlue.FRONT.trajectory.end())
                .setTangent(Math.toRadians(100))
                .splineToConstantHeading(
                        new Vector2d(36, 58),
                        Math.toRadians(90) //Tangent
                )
                .build()
        ),

        SECOND_PART(robot.autoDriveTrain.trajectorySequenceBuilder(FIRST_PART.trajectory.end())
                .splineToConstantHeading(
                        new Vector2d(36, 48),
                        Math.toRadians(90) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(36, 58),
                        Math.toRadians(90), //Tangent
                        robot.trajectories.reduceVelocity(0.4),
                        robot.trajectories.reduceAcceleration(0.4)
                )
                .build()
        );

        final TrajectorySequence trajectory;


        TrajectoriesBlue(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }
    }


}
