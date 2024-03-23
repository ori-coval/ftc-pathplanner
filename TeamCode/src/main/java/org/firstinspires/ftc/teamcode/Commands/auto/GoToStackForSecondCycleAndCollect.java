package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpMode;
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

    static RobotControl robot = AutonomousOpMode.robot;

    public GoToStackForSecondCycleAndCollect() {
        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryCommand(),
                        new ArmGetToPosition(robot, ArmPosition.INTAKE, true),
                        new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN),
                        new WaitCommand(700).andThen(
                                new InstantIntakeRotate(robot, robot.intake.roller.COLLECT_POWER),
                                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.THIRD_PIXEL)
                        )
                ),
                new CollectFromStack(robot, true)
        );
    }

    private Command getTrajectoryCommand() {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(RED, robot.autoDriveTrain),
                new TrajectoryFollowerCommand(BLUE, robot.autoDriveTrain),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }



    static final TrajectorySequence RED = robot.autoDriveTrain.trajectorySequenceBuilder(
            new Pose2d(
                    ScoringCommand.getCyclesRedTrajectory(robot).end().getX(),
                    TrajectoryPoses.realBackdropFront.getY(),
                    ScoringCommand.getCyclesRedTrajectory(robot).end().getHeading()
            )
            )
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        new Vector2d(-36, 60.5),
                        Math.toRadians(90)
                )
                .build();
    static final TrajectorySequence BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(ScoringCommand.getCyclesBlueTrajectory(robot).end())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        new Vector2d(36, 65),
                        Math.toRadians(90)
                )
                .build();

}
