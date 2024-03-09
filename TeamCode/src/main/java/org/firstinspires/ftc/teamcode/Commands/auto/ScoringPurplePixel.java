package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

public class ScoringPurplePixel extends ParallelCommandGroup {

    public ScoringPurplePixel(RobotControl robot) {
        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryCommand(robot),
//                        new WaitCommand(2000).andThen(new IntakeRotate(robot.intake.roller, robot.intake.roller.COLLECT_POWER).withTimeout(500))
                        new WaitCommand(2500) //todo remove later
                ),
                new WaitCommand(300).andThen(new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL, false))
        );
    }

    private Command getTrajectoryCommand(RobotControl robot) {
        return new ConditionalCommand(
                new ConditionalCommand(
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Far Detected) Red"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Center Detected) Red"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Close Detected) Red"), robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple (Far Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple (Center Detected) Red"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Close Purple (Close Detected) Red"), robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        () -> robot.robotSide == AllianceSide.FAR
                ),
                new DetectionSideCommandSwitch(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Far Detected) Blue"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Center Detected) Blue"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Far Purple (Close Detected) Blue"), robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }


}
