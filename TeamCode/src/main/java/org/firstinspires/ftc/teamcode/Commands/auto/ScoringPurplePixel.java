package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;

public class ScoringPurplePixel extends ParallelCommandGroup {

    private final long WAIT_UNTIL_EJECT_BACK = 2000;

    public ScoringPurplePixel(RobotControl robot) {
        addCommands(
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Left"), robot.driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Center"), robot.driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Right"), robot.driveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new WaitCommand(300).andThen(new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL, false)),
                new WaitCommand(1000).andThen(
                        new IntakeRotate(robot.intake.roller, robot.intake.roller.COLLECT_POWER).withTimeout(2000)
                )
        );
    }
}
