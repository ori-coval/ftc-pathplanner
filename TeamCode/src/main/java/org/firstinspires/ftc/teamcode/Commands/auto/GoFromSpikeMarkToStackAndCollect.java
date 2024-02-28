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
    public GoFromSpikeMarkToStackAndCollect(RobotControl robot) {
        super(
                new ParallelCommandGroup(
                        new SideCommandSwitch(
                                new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Far Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Center Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Close Detected)"), robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        new WaitCommand(500).andThen(new BackToIntake(robot).alongWith(new InstantCommand(() -> robot.intake.roller.setPower(robot.intake.roller.COLLECT_POWER)))),
                        new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL)
                ),
                new WaitCommand(1000).andThen(new InstantCommand(() -> robot.intake.roller.stop()))
        );
    }
}
