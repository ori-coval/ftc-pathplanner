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
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryPoses;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetLifterPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.InstantIntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class GoFromSpikeMarkToStackAndCollect extends SequentialCommandGroup {

    static RobotControl robot;

    public GoFromSpikeMarkToStackAndCollect(RobotControl robot) {
        GoFromSpikeMarkToStackAndCollect.robot = robot;
        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryCommand(),
                        new WaitCommand(200).andThen(
                                new ArmGetToPosition(robot, ArmPosition.INTAKE, true),
                                new WaitCommand(300),
                                new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN)
                        ),
                        new InstantIntakeRotate(robot, robot.intake.roller.COLLECT_POWER),
                        new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL)
                ),
                new CollectFromStack(robot)
        );
    }


    private Command getTrajectoryCommand() {
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


    //This trajectory is meant to drive to stack depending on the prop detected and the alliance color.

    public enum TrajectoriesRed {

        FAR(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesRed.FAR_FAR_PURPLE.trajectory.end())
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(
                        TrajectoryPoses.stackPoseRed,
                        Math.toRadians(45), //Tangent
                        robot.trajectories.reduceVelocity(0.6),
                        robot.trajectories.reduceAcceleration(0.6)
                )
                .build()
        ),

        CENTER(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesRed.FAR_CENTER_PURPLE.trajectory.end())
                .setTangent(Math.toRadians(40))
                .splineToSplineHeading(
                        new Pose2d(TrajectoryPoses.stackPoseRed.getX(), 47, Math.toRadians(90)),
                        Math.toRadians(90), //Tangent
                        robot.trajectories.reduceVelocity(0.8),
                        robot.trajectories.reduceAcceleration(0.8)
                )
                .splineToSplineHeading(
                        TrajectoryPoses.stackPoseRed,
                        Math.toRadians(90), //Tangent
                        robot.trajectories.reduceVelocity(0.5),
                        robot.trajectories.reduceAcceleration(0.5)
                )
                .build()
        ),

        CLOSE(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesRed.FAR_CLOSE_PURPLE.trajectory.end())
                .setTangent(Math.toRadians(45))
                .splineToConstantHeading(
                        new Vector2d(-22, 40),
                        Math.toRadians(75) //Tangent
                )
                .splineToSplineHeading(
                        TrajectoryPoses.stackPoseRed,
                        Math.toRadians(45), //Tangent
                        robot.trajectories.reduceVelocity(0.5),
                        robot.trajectories.reduceAcceleration(0.5)
                )
                .build()
        );

        final TrajectorySequence trajectory;

        TrajectoriesRed(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }
    }

    public enum TrajectoriesBlue {

        FAR(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesBlue.FAR_FAR_PURPLE.trajectory.end())
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(
                        TrajectoryPoses.stackPoseBlue,
                        Math.toRadians(135), //Tangent
                        robot.trajectories.reduceVelocity(0.6),
                        robot.trajectories.reduceAcceleration(0.6)
                )
                .build()
        ),
        CENTER(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesBlue.FAR_CENTER_PURPLE.trajectory.end())
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(
                        new Pose2d(TrajectoryPoses.stackPoseBlue.getX(), 47, Math.toRadians(90)),
                        Math.toRadians(90), //Tangent
                        robot.trajectories.reduceVelocity(0.8),
                        robot.trajectories.reduceAcceleration(0.8)
                )
                .splineToSplineHeading(
                        TrajectoryPoses.stackPoseRed,
                        Math.toRadians(90), //Tangent
                        robot.trajectories.reduceVelocity(0.5),
                        robot.trajectories.reduceAcceleration(0.5)
                )
                .build()
        ),
        CLOSE(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringPurplePixel.TrajectoriesBlue.FAR_CLOSE_PURPLE.trajectory.end())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        new Vector2d(22, 40),
                        Math.toRadians(135) //Tangent
                )
                .splineToSplineHeading(
                        TrajectoryPoses.stackPoseBlue,
                        Math.toRadians(90), //Tangent
                        robot.trajectories.reduceVelocity(0.5),
                        robot.trajectories.reduceAcceleration(0.5)
                )
                .build()
        );

        final TrajectorySequence trajectory;

        TrajectoriesBlue(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }

    }

}
