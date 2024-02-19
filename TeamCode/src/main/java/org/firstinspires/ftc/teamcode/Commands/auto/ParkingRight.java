package org.firstinspires.ftc.teamcode.Commands.auto;

import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;

public class ParkingRight extends SideCommandSwitch {
    public ParkingRight(RobotControl robot) {
        super(
                new TrajectoryFollowerCommand(Trajectories.get("Parking right (left)"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(Trajectories.get("Parking right (center)"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(Trajectories.get("Parking right (right)"), robot.autoDriveTrain),
                () -> robot.robotSide
        );
    }
}
