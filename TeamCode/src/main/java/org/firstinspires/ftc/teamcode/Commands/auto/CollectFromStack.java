package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetLifterPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeStop;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class CollectFromStack extends ParallelCommandGroup {
    public CollectFromStack(RobotControl robot) {
        addCommands(
                addBite(robot),
                new SequentialCommandGroup(
                        new WaitUntilCommand(
                                robot.intake.roller::isRobotFull
                        ).withTimeout(2500),
                        stopAndCloseCartridge(robot)
                )
        );
    }

    public CollectFromStack(RobotControl robot, boolean secondCycle) {
        addCommands(
                new SequentialCommandGroup(
                        new WaitUntilCommand(
                                () -> robot.intake.roller.getPixelCount() == 1
                        ).withTimeout(1000),
                        new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.THIRD_FOURTH_PIXELS),
                        new WaitUntilCommand(
                                robot.intake.roller::isRobotFull
                        ).withTimeout(1500),
                        stopAndCloseCartridge(robot)
                )
        );
    }

    private Command addBite(RobotControl robot) {
            return new SequentialCommandGroup(
                    new TrajectoryFollowerCommand(robot.trajectories.get("Drive back from stack"), robot.autoDriveTrain),
                    new TrajectoryFollowerCommand(robot.trajectories.get("Drive back to stack"), robot.autoDriveTrain)
            );
    }

    private Command stopAndCloseCartridge(RobotControl robot) {
        return new SequentialCommandGroup(
                new WaitCommand(1000),
                new IntakeStop(robot),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
        );
    }


}
