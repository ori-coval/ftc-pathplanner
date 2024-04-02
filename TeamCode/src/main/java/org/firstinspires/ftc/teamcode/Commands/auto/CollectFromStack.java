package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetLifterPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeStop;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class CollectFromStack extends ParallelCommandGroup {
    static RobotControl robot = AutonomousOpMode.robot;
    boolean canStop = false;

    public CollectFromStack(RobotControl robot) {
        CollectFromStack.robot = robot;
        addCommands(new ConditionalCommand(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(robot.intake.roller::isRobotFull).interruptOn(() -> canStop),
                                stopAndCloseCartridge(robot)
                        ),
                        new SequentialCommandGroup(
                                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.SECOND_PIXEL),
                                addBite(robot, 1),
                                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT),
                                addBite(robot, 2),
                                new WaitCommand(300),
                                new InstantCommand(() -> canStop = true)
                        )
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(robot.intake.roller::isRobotFull).interruptOn(() -> canStop),
                                stopAndCloseCartridge(robot)
                        ),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        addBite(robot, 2),
                                        new WaitCommand(200).andThen(
                                                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.SECOND_PIXEL),
                                                new WaitCommand(1000),
                                                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT)
                                        )
                                ),
                                new WaitCommand(500),
                                new InstantCommand(() -> canStop = true)
                        )
                ),
                () -> robot.allianceColor == AllianceColor.RED
        ));
    }

    public CollectFromStack(RobotControl robot, boolean secondCycle) {
        addCommands(
                addBite(robot, 1),
                new SequentialCommandGroup(
                        new WaitUntilCommand(
                                () -> robot.intake.roller.getPixelCount() == 1
                        ).withTimeout(300),
                        new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.FOURTH_PIXEL),
                        new WaitUntilCommand(
                                robot.intake.roller::isRobotFull
                        ).withTimeout(1000),
                        stopAndCloseCartridge(robot)
                )
        );
    }

    private Command addBite(RobotControl robot, int numOfBite) {
            return new ConditionalCommand(
                    new ConditionalCommand(
                            new TrajectoryFollowerCommand(FIRST_BITE_RED, robot.autoDriveTrain),
                            new TrajectoryFollowerCommand(SECOND_BITE_RED, robot.autoDriveTrain),
                            () -> numOfBite == 1
                    ),
                    new ConditionalCommand(
                            new TrajectoryFollowerCommand(FIRST_BITE_BLUE, robot.autoDriveTrain),
                            new ConditionalCommand(
                                    new TrajectoryFollowerCommand(SECOND_BITE_FAR_BLUE, robot.autoDriveTrain),
                                    new TrajectoryFollowerCommand(SECOND_BITE_BLUE, robot.autoDriveTrain),
                                    () -> robot.teamPropDetector.getTeamPropSide() == DetectionSide.FAR
                            ),
                            () -> numOfBite == 1
                    ),
                    () -> robot.allianceColor == AllianceColor.RED
            );
    }

    static TrajectorySequence FIRST_BITE_RED = robot.autoDriveTrain.trajectorySequenceBuilder(GoToStackForFirstCycleAndCollect.SECOND_PART_RED.end())
            .back(7)
            .splineToConstantHeading(
                    new Vector2d(
                            Trajectories.stackPoseRed.getX(),
                            Trajectories.stackPoseRed.getY() + 4
                    ),
                    Math.toRadians(90),
                    Trajectories.reduceVelocity(0.6),
                    Trajectories.reduceAcceleration(0.6)
            )
            .build();

    static TrajectorySequence SECOND_BITE_RED = robot.autoDriveTrain.trajectorySequenceBuilder(FIRST_BITE_RED.end())
            .back(12)
            .splineToConstantHeading(
                    new Vector2d(
                            Trajectories.stackPoseRed.getX(),
                            Trajectories.stackPoseRed.getY() + 3
                    ),
                    Math.toRadians(90),
                    Trajectories.reduceVelocity(0.4),
                    Trajectories.reduceAcceleration(0.4)
            )
            .build();

    static TrajectorySequence FIRST_BITE_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(GoToStackForFirstCycleAndCollect.SECOND_PART_CENTER_BLUE.end())
            .back(5)
            .splineToConstantHeading(
                    new Vector2d(
                            Trajectories.stackPoseBlue.getX() - 1,
                            Trajectories.stackPoseBlue.getY() + 4
                    ),
                    Math.toRadians(90),
                    Trajectories.reduceVelocity(0.5),
                    Trajectories.reduceAcceleration(0.5)
            )
            .build();
    static TrajectorySequence SECOND_BITE_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.stackPoseBlue)
            .back(10)
            .splineToConstantHeading(
                    new Vector2d(
                            Trajectories.stackPoseBlue.getX() - 1,
                            Trajectories.stackPoseBlue.getY() + 3
                    ),
                    Math.toRadians(90),
                    Trajectories.reduceVelocity(0.25),
                    Trajectories.reduceAcceleration(0.25)
            )
            .build();
    static TrajectorySequence SECOND_BITE_FAR_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.stackPoseBlue)
            .back(10)
            .splineToConstantHeading(
                    new Vector2d(
                            Trajectories.stackPoseBlue.getX() - 4,
                            Trajectories.stackPoseBlue.getY() - 5
                    ),
                    Math.toRadians(90),
                    Trajectories.reduceVelocity(0.25),
                    Trajectories.reduceAcceleration(0.25)
            )
            .build();

    private Command stopAndCloseCartridge(RobotControl robot) {
        return new SequentialCommandGroup(
                new WaitCommand(200),
                new IntakeStop(robot),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
        );
    }


}
