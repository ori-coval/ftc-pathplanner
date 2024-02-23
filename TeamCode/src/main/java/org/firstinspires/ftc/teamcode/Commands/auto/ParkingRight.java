package org.firstinspires.ftc.teamcode.Commands.auto;

import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;

public class ParkingRight extends SideCommandSwitch {
    public ParkingRight(RobotControl robot) {
        super(
                new TrajectoryFollowerCommand(robot.trajectories.get("Parking right (left)"), robot.driveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Parking right (center)"), robot.driveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Parking right (right)"), robot.driveTrain),
                () -> robot.robotSide
        );
    }
}
