package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class TakingFirstPixelFromStack extends ParallelRaceGroup {
    RobotControl robot;
    public TakingFirstPixelFromStack(RobotControl robot) {
        this.robot = robot;
        addCommands(
                new WaitUntilCommand(robot.intake.roller::isRobotFull),
                new SequentialCommandGroup(
                        getBackAndForthCommand(),
                        new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.SECOND_PIXEL),
                        getBackAndForthCommand(),
                        new TrajectoryFollowerCommand(Trajectories.get("Drive back from stack"), robot.autoDriveTrain)
                )
        );
    }

    public SequentialCommandGroup getBackAndForthCommand() {
        return new SequentialCommandGroup(
                new TrajectoryFollowerCommand(Trajectories.get("Drive back from stack"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(Trajectories.get("Drive back to stack"), robot.autoDriveTrain)
        );
    }

}
