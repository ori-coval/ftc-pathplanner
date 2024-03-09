package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class Parking extends ParallelCommandGroup {
    public Parking(RobotControl robot) {
        addCommands(
                new ConditionalCommand(
                        getFarTrajectory(robot),
                        getCloseTrajectory(robot),
                        () -> robot.robotSide == AllianceSide.FAR
                ), //to allow intake to get in
                new WaitCommand(500).andThen(new ArmGetToPosition(robot, ArmPosition.INTAKE, false))
        );
    }

    private Command getFarTrajectory(RobotControl robot) {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(robot.trajectories.get("Parking Arm To Intake (Far Side) Red"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Parking Arm To Intake (Far Side) Blue"), robot.autoDriveTrain),
                () -> robot.allianceColor == AllianceColor.RED
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
