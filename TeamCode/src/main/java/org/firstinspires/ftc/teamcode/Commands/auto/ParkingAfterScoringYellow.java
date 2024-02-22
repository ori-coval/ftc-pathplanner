package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;

public class ParkingAfterScoringYellow extends SequentialCommandGroup {
    public ParkingAfterScoringYellow(RobotControl robot) {
        super(
                new TrajectoryFollowerCommand(Trajectories.get("Go back after scoring yellow"), robot.driveTrain), //to allow intake to get in
                new ArmGetToPosition(robot, ArmPosition.INTAKE, true)
        );
    }
}
