package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeStop;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

public class CollectFromStack extends ParallelCommandGroup {
    public CollectFromStack(RobotControl robot) {
        super(
                new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Drive back from stack"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Drive back to stack"), robot.autoDriveTrain)
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(robot.intake.roller::isRobotFull).withTimeout(2500),
                        new WaitCommand(1000),
                        new IntakeStop(robot),
                        new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
                )
        );
    }
}
