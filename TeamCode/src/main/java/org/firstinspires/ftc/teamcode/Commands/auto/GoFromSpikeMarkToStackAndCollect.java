package org.firstinspires.ftc.teamcode.Commands.auto;


import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class GoFromSpikeMarkToStackAndCollect extends SequentialCommandGroup {
    public GoFromSpikeMarkToStackAndCollect(RobotControl robot) {
        super(
                new ParallelCommandGroup(
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Far Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Center Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Close Detected)"), robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        new WaitCommand(200).andThen(new BackToIntake(robot)),
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new WaitUntilCommand(robot.intake.roller::isRobotFull).andThen(new WaitCommand(100)),
                                        new IntakeRotate(robot.intake.roller, robot.intake.roller.COLLECT_POWER),
                                        new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL)
                                ).asProxy(),
                                new IntakeRotate(robot.intake.roller, robot.intake.roller.EJECT_POWER).withTimeout(500)
                        )
                )
        );
    }
}
