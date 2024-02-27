package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.Side;

public class ParkingAfterScoringYellow extends SequentialCommandGroup {
    public ParkingAfterScoringYellow(RobotControl robot) {
        addCommands(
                new ConditionalCommand(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Backdrop Intake Far"), robot.autoDriveTrain),
                        getCloseTrajectory(robot),
                        () -> robot.robotSide == AllianceSide.FAR
                ), //to allow intake to get in
                new InstantCommand(() -> {
                    robot.telemetry.addLine(String.valueOf(robot.robotSide));
                    robot.telemetry.update();
                })/*,
                new ArmGetToPosition(robot, ArmPosition.INTAKE, robot.allianceColor == AllianceColor.RED)*/
        );
    }

    private Command getCloseTrajectory(RobotControl robot) {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(robot.trajectories.get("Backdrop Intake Close Problematic"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Backdrop Intake Close"), robot.autoDriveTrain),
                () -> (robot.allianceColor == AllianceColor.RED && robot.teamPropDetector.getTeamPropSide() == Side.LEFT)
                ||    (robot.allianceColor == AllianceColor.BLUE && robot.teamPropDetector.getTeamPropSide() == Side.RIGHT)
        );
    }

}
