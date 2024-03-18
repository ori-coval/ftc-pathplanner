package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

public class GoToStackForSecondCycleAndCollect extends SequentialCommandGroup {

    static RobotControl robot;

    public GoToStackForSecondCycleAndCollect(RobotControl robot) {
        GoToStackForSecondCycleAndCollect.robot = robot;
        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryCommandPart1(robot),
                        new ArmGetToPosition(robot, ArmPosition.SCORING_AUTO, robot.allianceColor == AllianceColor.RED).andThen(
                                new ArmGetToPosition(robot, ArmPosition.INTAKE, true)
                        ),
                        new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
                ),
                new ParallelCommandGroup(
                        getTrajectoryCommandPart2(robot),
                        new WaitCommand(300).andThen(new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN)),
                        new WaitCommand(500).andThen(
                                new InstantIntakeRotate(robot, robot.intake.roller.COLLECT_POWER),
                                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.THIRD_PIXEL)
                        )
                ),
                new CollectFromStack(robot, true)
        );
    }

    private Command getTrajectoryCommandPart1(RobotControl robot) {
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
                        new Pose2d(TrajectoryPoses.stackPoseRed.getX() + 3, -15, Math.toRadians(90)),
                        Math.toRadians(95) //Tangent
                )
                .build()
        ),
        SECOND_PART(robot.autoDriveTrain.trajectorySequenceBuilder(FIRST_PART.trajectory.end())
                .splineToLinearHeading(
                        new Pose2d(TrajectoryPoses.stackPoseRed.getX(), 48, Math.toRadians(90)),
                        Math.toRadians(85) //Tangent
                )
                .splineToLinearHeading(
                        new Pose2d(-14, 58, Math.toRadians(90)), //annoying stack red
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
                        new Pose2d(TrajectoryPoses.stackPoseBlue.getX() - 2, -15, Math.toRadians(90)),
                        Math.toRadians(85) //Tangent
                )
                .build()
        ),
        SECOND_PART(robot.autoDriveTrain.trajectorySequenceBuilder(FIRST_PART.trajectory.end())
                .setTangent(Math.toRadians(85))
                .splineToLinearHeading(
                        new Pose2d(TrajectoryPoses.stackPoseBlue.getX(), 48, Math.toRadians(90)),
                        Math.toRadians(90) //Tangent
                )
                .splineToLinearHeading(
                        TrajectoryPoses.stackPoseBlue,
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
