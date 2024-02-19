package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;

public class ScoringPurplePixel extends SequentialCommandGroup {

    private final long WAIT_UNTIL_EJECT_BACK = 2000;

    public ScoringPurplePixel(RobotControl robot) {
        addCommands(
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Left"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Center"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Right"), robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL_RIGHT, false),
                new IntakeRotate(robot.intake.roller, robot.intake.roller.COLLECT_POWER).withTimeout(WAIT_UNTIL_EJECT_BACK)
        );
    }
}
