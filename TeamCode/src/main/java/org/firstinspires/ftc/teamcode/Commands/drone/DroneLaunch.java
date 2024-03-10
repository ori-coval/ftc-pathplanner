package org.firstinspires.ftc.teamcode.Commands.drone;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotControl;

public class DroneLaunch extends SequentialCommandGroup {

    private final long TIME_BETWEEN = 300;

    public DroneLaunch(RobotControl robot) {
        addCommands(
                setPos(robot, 0.5),
                setPos(robot, 0.3),
                setPos(robot, 0.6)
        );
    }

    private Command setPos(RobotControl robot, double pos) {
        return new SequentialCommandGroup(
                new DroneLauncherSetPosition(robot.droneLauncher, pos),
                new WaitCommand(TIME_BETWEEN)
        );
    }

}
