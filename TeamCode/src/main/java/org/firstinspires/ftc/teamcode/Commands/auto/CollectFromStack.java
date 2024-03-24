package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryPoses;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetLifterPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeStop;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class CollectFromStack extends ParallelCommandGroup {
    static RobotControl robot = AutonomousOpMode.robot;
    public CollectFromStack(RobotControl robot) {
        CollectFromStack.robot = robot;
        addCommands(
                addBite(robot),
                new SequentialCommandGroup(
                        new WaitUntilCommand(
                                () -> robot.intake.roller.getPixelCount() == 1
                        ).withTimeout(300),
                        new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.SECOND_PIXEL),
                        new WaitUntilCommand(
                                robot.intake.roller::isRobotFull
                        ).withTimeout(1000),
                        stopAndCloseCartridge(robot)
                )
        );
    }

    public CollectFromStack(RobotControl robot, boolean secondCycle) {
        addCommands(
                addBite(robot),
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

    private Command addBite(RobotControl robot) {
            return new ConditionalCommand(
                    new SequentialCommandGroup(
                            new TrajectoryFollowerCommand(BITE_RED, robot.autoDriveTrain)
                    ),
                    new SequentialCommandGroup(
                            new TrajectoryFollowerCommand(BITE_BLUE, robot.autoDriveTrain)
                    ),
                    () -> robot.allianceColor == AllianceColor.RED
            );
    }

    static TrajectorySequence BITE_RED = robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.stackPoseRed)
            .back(2)
            .splineToConstantHeading(
                    TrajectoryPoses.stackPoseRed.vec(),
                    Math.toRadians(90),
                    robot.trajectories.reduceVelocity(0.4),
                    robot.trajectories.reduceAcceleration(0.4)
            )
            .build();

    static TrajectorySequence BITE_BLUE = robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.stackPoseBlue)
            .back(2)
            .splineToConstantHeading(
                    TrajectoryPoses.stackPoseBlue.vec(),
                    Math.toRadians(90)
            )
            .build();

    private Command stopAndCloseCartridge(RobotControl robot) {
        return new SequentialCommandGroup(
                new WaitCommand(300),
                new IntakeStop(robot),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
        );
    }


}
