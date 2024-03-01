package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class Parking extends SequentialCommandGroup {
    public Parking(RobotControl robot) {
        addCommands(
                new ConditionalCommand(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Parking Arm To Intake (Far Side)"), robot.autoDriveTrain),
                        getCloseTrajectory(robot),
                        () -> robot.robotSide == AllianceSide.FAR
                ), //to allow intake to get in
                new ArmGetToPosition(robot, ArmPosition.INTAKE, false)
        );
    }

    private Command getCloseTrajectory(RobotControl robot) {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(robot.trajectories.get("Backdrop Intake Close (Close Detected)"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Backdrop Intake Close"), robot.autoDriveTrain),
                () ->  robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
        );
    }

}
