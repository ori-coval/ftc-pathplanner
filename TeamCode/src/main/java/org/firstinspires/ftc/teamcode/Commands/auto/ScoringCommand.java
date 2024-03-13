package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
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
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

import java.util.function.Supplier;

public class ScoringCommand extends SequentialCommandGroup {

    static RobotControl robot;

    public ScoringCommand(Command scoringCommand, Command secondScoringCommand, RobotControl robot, Double numOfCycle) {
        ScoringCommand.robot = robot;
        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryCommand(robot),
                        new IntakeRotate(robot.intake.roller, robot.intake.roller.EJECT_POWER).withTimeout(1500),
                        new WaitCommand(1700).andThen(
                                new ArmGetToPosition(robot, ArmPosition.SCORING, robot.allianceColor == AllianceColor.RED),
                                new InstantCommand(() -> RotateTurretByPID.DEADLINE_FOR_TURRET = 700),//todo maybe will work with less time
                                scoringCommand
                        )
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new CartridgeSetState(robot.cartridge, Cartridge.State.SEMI_OPEN),
                                new WaitCommand(200),
                                new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN)
                        ),
                        new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN),
                        () -> numOfCycle == 1
                ),
                new ResetPixelCount(robot),
                new InstantCommand(() -> RotateTurretByPID.DEADLINE_FOR_TURRET = 2000),
                new WaitCommand(300)
// <- todo remove this when we need elevator               secondScoringCommand
        );
    }


    private Command getTrajectoryCommand(RobotControl robot) {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(Trajectories.RED.trajectory, robot.autoDriveTrain).andThen(resetPoseEstimate(robot)),
                new TrajectoryFollowerCommand(Trajectories.BLUE.trajectory, robot.autoDriveTrain).andThen(resetPoseEstimate(robot)),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private Command resetPoseEstimate(RobotControl robot) {
        return new ConditionalCommand(
                new InstantCommand(() ->
                        robot.autoDriveTrain.setPoseEstimate(new Pose2d(
                                TrajectoryPoses.realBackdropPoseRed.getX(),
                                TrajectoryPoses.realBackdropPoseRed.getY(),
                                robot.autoDriveTrain.getPoseEstimate().getHeading()
                        ))
                ),
                new InstantCommand(() ->
                    robot.autoDriveTrain.setPoseEstimate(new Pose2d(
                            TrajectoryPoses.realBackdropPoseBlue.getX(),
                            TrajectoryPoses.realBackdropPoseBlue.getY(),
                            robot.autoDriveTrain.getPoseEstimate().getHeading()
                    ))
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    //Go to backdrop depending on alliance color. //todo front scoring

    public enum Trajectories {

        RED(robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.stackPoseRed)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(
                        new Pose2d(TrajectoryPoses.stackPoseRed.getX() + 3, -15, Math.toRadians(90)),
                        Math.toRadians(-90) //Tangent
                )
                .splineToLinearHeading(
                        new Pose2d(-9, -40, Math.toRadians(90)),
                        Math.toRadians(-90) //Tangent
                )
                .splineToLinearHeading(
                        new Pose2d(-15, -61, Math.toRadians(90)),
                        Math.toRadians(-140) //Tangent
                )
                .splineToLinearHeading(
                        new Pose2d(-22, -62, Math.toRadians(90)),
                        Math.toRadians(180), //Tangent
                        robot.trajectories.reduceVelocity(0.6),
                        robot.trajectories.reduceAcceleration(0.6)
                )
                .build()
        ),
        BLUE(robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.stackPoseBlue)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(
                        new Pose2d(TrajectoryPoses.stackPoseBlue.getX() - 2, -15, Math.toRadians(90)),
                        Math.toRadians(270) //Tangent
                )
                .splineToLinearHeading(
                        new Pose2d(9, -40, Math.toRadians(90)),
                        Math.toRadians(270) //Tangent
                )
                .splineToLinearHeading(
                        new Pose2d(15, -58, Math.toRadians(90)),
                        Math.toRadians(320) //Tangent
                )
                .splineToLinearHeading(
                        new Pose2d(28, -63, Math.toRadians(90)),
                        Math.toRadians(-20), //Tangent
                        robot.trajectories.reduceVelocity(0.6),
                        robot.trajectories.reduceAcceleration(0.6)
                )
                .build()
        );

        final TrajectorySequence trajectory;

        Trajectories(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }

    }


}
