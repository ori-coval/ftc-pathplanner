package org.firstinspires.ftc.teamcode.Commands.auto;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class GoFromSpikeMarkToStackAndCollect extends SequentialCommandGroup {
    public GoFromSpikeMarkToStackAndCollect(RobotControl robot, AllianceColor allianceColor) {
        super(
                new ParallelCommandGroup(
                        new ConditionalCommand(
                                new SideCommandSwitch(
                                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack left"), robot.driveTrain),
                                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack center"), robot.driveTrain),
                                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack right"), robot.driveTrain),
                                        () -> robot.teamPropDetector.getTeamPropSide()
                                ),
                                new SideCommandSwitch(
                                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack right"), robot.driveTrain),
                                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack center"), robot.driveTrain),
                                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack left"), robot.driveTrain),
                                        () -> robot.teamPropDetector.getTeamPropSide()
                                ),
                                () -> allianceColor == AllianceColor.RED
                        ),
                        new WaitCommand(500).andThen(new BackToIntake(robot).alongWith(new InstantCommand(() -> robot.intake.roller.setPower(robot.intake.roller.COLLECT_POWER)))),
                        new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL)
                ),
                new WaitCommand(1000).andThen(new InstantCommand(() -> robot.intake.roller.stop()))
        );
    }
}
