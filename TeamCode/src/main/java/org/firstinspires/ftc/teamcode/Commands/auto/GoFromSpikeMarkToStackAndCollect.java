package org.firstinspires.ftc.teamcode.Commands.auto;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class GoFromSpikeMarkToStackAndCollect extends SequentialCommandGroup {
    public GoFromSpikeMarkToStackAndCollect(RobotControl robot) {
        super(
                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL),
                new InstantCommand(() -> robot.intake.roller.setPower(robot.intake.roller.COLLECT_POWER)),
                new SideCommandSwitch(
                        new SequentialCommandGroup(
                                new TrajectoryFollowerCommand(Trajectories.get("loading intake left"), robot.autoDriveTrain),
                                new BackToIntake(robot),
                                new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Left"), robot.autoDriveTrain)
                        ),
                        new InstantCommand(),
                        new InstantCommand(),
                        /*
                        new SequentialCommandGroup(
                                new TrajectoryFollowerCommand(Trajectories.get("loading intake center"), robot.autoDriveTrain),
                                new BackToIntake(robot),
                                new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Center"), robot.autoDriveTrain)
                        ),
                        new SequentialCommandGroup(
                                new TrajectoryFollowerCommand(Trajectories.get("loading intake right"), robot.autoDriveTrain),
                                new BackToIntake(robot),
                                new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Right"), robot.autoDriveTrain)
                        ),*/
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new TrajectoryFollowerCommand(Trajectories.get("Drive back from stack"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(Trajectories.get("Drive back to stack"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(Trajectories.get("Drive back from stack"), robot.autoDriveTrain),
                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.SECOND_PIXEL),
                new TrajectoryFollowerCommand(Trajectories.get("Drive back to stack"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(Trajectories.get("Drive back from stack"), robot.autoDriveTrain),
                new InstantCommand(() -> robot.intake.roller.stop())
        );
    }
}
